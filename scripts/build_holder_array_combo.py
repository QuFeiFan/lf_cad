#!/usr/bin/env python3
"""Build tilted holder array, cut one plate, and export assembly with visual guides."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

import cadquery as cq


Vec3 = Tuple[float, float, float]
Vec2 = Tuple[float, float]


def load_config(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def create_box(size_mm: List[float], center_mm: List[float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_mm[0], size_mm[1], size_mm[2]).translate(tuple(center_mm))


def resolve_plane_center(component_cfg: Dict, default_xy: Vec2 = (0.0, 0.0)) -> List[float]:
    size = component_cfg["size_mm"]
    if "front_z_mm" in component_cfg:
        if "xy_center_mm" in component_cfg:
            cx, cy = component_cfg["xy_center_mm"]
        elif "center_mm" in component_cfg:
            cx, cy = component_cfg["center_mm"][0], component_cfg["center_mm"][1]
        else:
            cx, cy = default_xy
        cz = float(component_cfg["front_z_mm"]) - float(size[2]) / 2.0
        return [float(cx), float(cy), cz]

    if "center_mm" not in component_cfg:
        raise ValueError("component config requires center_mm or front_z_mm")
    cx, cy, cz = component_cfg["center_mm"]
    return [float(cx), float(cy), float(cz)]


def resolve_plane_front_z(component_cfg: Dict) -> float:
    if "front_z_mm" in component_cfg:
        return float(component_cfg["front_z_mm"])
    if "center_mm" in component_cfg:
        return float(component_cfg["center_mm"][2]) + float(component_cfg["size_mm"][2]) / 2.0
    raise ValueError("component config requires center_mm or front_z_mm")


def normalize_dir(v: Vec3) -> Vec3:
    vx, vy, vz = v
    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if norm < 1e-9:
        raise ValueError("invalid direction with near-zero magnitude")
    return (vx / norm, vy / norm, vz / norm)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    ax, ay, az = a
    bx, by, bz = b
    return (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def vector_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vector_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def load_holder_template(config: Dict, config_path: Path) -> cq.Workplane:
    holder_cfg = config.get("lens_holder", {})
    if not holder_cfg.get("enabled", False):
        raise ValueError("lens_holder.enabled is false")

    step_path_cfg = holder_cfg.get("step_path")
    if not step_path_cfg:
        raise ValueError("lens_holder.step_path is required")

    step_path = Path(step_path_cfg)
    if not step_path.is_absolute():
        candidates = [
            (config_path.parent / step_path).resolve(),
            (config_path.parent.parent / step_path).resolve(),
            (Path.cwd() / step_path).resolve(),
        ]
        existing = [p for p in candidates if p.exists()]
        if not existing:
            raise FileNotFoundError(
                "lens_holder STEP not found in any candidate path: "
                + ", ".join(str(p) for p in candidates)
            )
        step_path = existing[0]
    elif not step_path.exists():
        raise FileNotFoundError(f"lens_holder STEP not found: {step_path}")

    holder_wp = cq.importers.importStep(str(step_path))

    if holder_cfg.get("normalize_import_center", True):
        bb = holder_wp.val().BoundingBox()
        holder_wp = holder_wp.translate((-bb.center.x, -bb.center.y, -bb.center.z))

    return holder_wp


def get_cylindrical_faces(shape: cq.Shape) -> List[Dict]:
    cylinders: List[Dict] = []
    for face in shape.Faces():
        if face.geomType() != "CYLINDER":
            continue

        cyl = face._geomAdaptor().Cylinder()
        axis = cyl.Axis()
        axis_loc = axis.Location()
        axis_dir = axis.Direction()

        cylinders.append(
            {
                "area": float(face.Area()),
                "radius": float(cyl.Radius()),
                "axis_loc": (float(axis_loc.X()), float(axis_loc.Y()), float(axis_loc.Z())),
                "axis_dir": (float(axis_dir.X()), float(axis_dir.Y()), float(axis_dir.Z())),
            }
        )

    return cylinders


def dedupe_cylinder_axes(cylinders: List[Dict], tol: float = 1e-4) -> List[Dict]:
    unique: List[Dict] = []
    seen = set()

    for c in cylinders:
        dx, dy, dz = normalize_dir(c["axis_dir"])

        if (dx < 0) or (abs(dx) <= tol and dy < 0) or (abs(dx) <= tol and abs(dy) <= tol and dz < 0):
            dx, dy, dz = -dx, -dy, -dz

        lx, ly, lz = c["axis_loc"]
        key = (
            round(c["radius"], 4),
            round(lx, 4),
            round(ly, 4),
            round(lz, 4),
            round(dx, 4),
            round(dy, 4),
            round(dz, 4),
        )
        if key in seen:
            continue
        seen.add(key)

        unique.append(
            {
                "radius": c["radius"],
                "axis_loc": (lx, ly, lz),
                "axis_dir": (dx, dy, dz),
                "area": c["area"],
            }
        )

    return unique


def select_outer_cylinder(cylinders: List[Dict], mode: str = "max_radius") -> Dict:
    if not cylinders:
        raise ValueError("No cylindrical faces found on holder")

    unique = dedupe_cylinder_axes(cylinders)

    if mode == "max_radius":
        target = max(c["radius"] for c in unique)
        selected = [c for c in unique if abs(c["radius"] - target) < 1e-4]
    elif mode == "max_area":
        target = max(c["area"] for c in unique)
        selected = [c for c in unique if abs(c["area"] - target) < 1e-4]
    else:
        raise ValueError(f"Unsupported cutter mode: {mode}")

    if not selected:
        raise ValueError("Failed to select outer cylinder")

    # For this holder model outer shell is unique after dedupe; keep first for axis anchoring.
    return selected[0]


def rotate_wp_about_center(wp: cq.Workplane, axis_dir: Vec3, angle_deg: float) -> cq.Workplane:
    bb = wp.val().BoundingBox()
    center = (float(bb.center.x), float(bb.center.y), float(bb.center.z))
    axis = normalize_dir(axis_dir)
    axis_end = vector_add(center, axis)
    return wp.rotate(center, axis_end, angle_deg)


def align_axis_to_target(wp: cq.Workplane, current_axis: Vec3, target_axis: Vec3) -> cq.Workplane:
    v1 = normalize_dir(current_axis)
    v2 = normalize_dir(target_axis)
    c = clamp(dot(v1, v2), -1.0, 1.0)

    if c > 1.0 - 1e-9:
        return wp

    if c < -1.0 + 1e-9:
        candidate = cross(v1, (1.0, 0.0, 0.0))
        if math.sqrt(dot(candidate, candidate)) < 1e-9:
            candidate = cross(v1, (0.0, 1.0, 0.0))
        return rotate_wp_about_center(wp, candidate, 180.0)

    rot_axis = cross(v1, v2)
    angle_deg = math.degrees(math.acos(c))
    return rotate_wp_about_center(wp, rot_axis, angle_deg)


def make_cutter_cylinder(axis_loc: Vec3, axis_dir: Vec3, radius: float, length: float) -> cq.Workplane:
    dx, dy, dz = normalize_dir(axis_dir)
    lx, ly, lz = axis_loc
    start = (lx - dx * length / 2.0, ly - dy * length / 2.0, lz - dz * length / 2.0)
    solid = cq.Solid.makeCylinder(radius=radius, height=length, pnt=start, dir=(dx, dy, dz))
    return cq.Workplane(obj=solid)


def make_segment_cylinder(start: Vec3, end: Vec3, radius: float) -> cq.Workplane | None:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = (ex - sx, ey - sy, ez - sz)
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-9:
        return None
    solid = cq.Solid.makeCylinder(radius=radius, height=length, pnt=start, dir=(dx, dy, dz))
    return cq.Workplane(obj=solid)


def extend_point_through_target_to_z(start: Vec3, through: Vec3, target_z: float) -> Vec3:
    sx, sy, sz = start
    tx, ty, tz = through
    dz = tz - sz
    if abs(dz) < 1e-9:
        raise ValueError("Cannot extend line to target z when start/through share same z")
    scale = (target_z - sz) / dz
    return (sx + (tx - sx) * scale, sy + (ty - sy) * scale, target_z)


def build_visual_guides(config: Dict, sub_centers_2d: List[Vec2], plate_points: List[Vec3]) -> cq.Workplane | None:
    combo_cfg = config.get("holder_array_combo", {})
    vis = combo_cfg.get("visualization", {})

    show_axes = bool(vis.get("show_convergence_axes", True))
    show_lines = bool(vis.get("show_sub_to_lens_lines", True))
    axis_radius = float(vis.get("axis_radius_mm", 1.2))
    line_radius = float(vis.get("line_radius_mm", 0.6))
    axis_length = float(vis.get("axis_length_mm", 500.0))
    lines_end_z = float(vis.get("sub_to_lens_lines_end_z_mm", 500.0))

    shapes: List[cq.Shape] = []

    if show_axes:
        conv = config["convergence_control"]
        z_v = float(conv["vertical_convergence_x_axis_line_z_mm"])
        z_h = float(conv["horizontal_convergence_y_axis_line_z_mm"])

        x_axis = make_segment_cylinder(
            (-axis_length / 2.0, 0.0, z_v),
            (axis_length / 2.0, 0.0, z_v),
            axis_radius,
        )
        y_axis = make_segment_cylinder(
            (0.0, -axis_length / 2.0, z_h),
            (0.0, axis_length / 2.0, z_h),
            axis_radius,
        )
        if x_axis is not None:
            shapes.append(x_axis.val())
        if y_axis is not None:
            shapes.append(y_axis.val())

    if show_lines:
        z_screen = resolve_plane_front_z(config["screen"])
        for (xs, ys), (xl, yl, zl) in zip(sub_centers_2d, plate_points):
            line_start = (xs, ys, z_screen)
            line_through = (xl, yl, zl)
            line_end = extend_point_through_target_to_z(line_start, line_through, lines_end_z)
            seg = make_segment_cylinder(line_start, line_end, line_radius)
            if seg is not None:
                shapes.append(seg.val())

    if not shapes:
        return None
    return cq.Workplane(obj=cq.Compound.makeCompound(shapes))


def trim_holder_keep_front_by_z_plane(
    holder_wp: cq.Workplane,
    z_plane: float,
    clip_xy_mm: float = 5000.0,
    clip_z_mm: float = 5000.0,
) -> cq.Workplane:
    # Keep the +Z side relative to the cut plane.
    clip_box = (
        cq.Workplane("XY")
        .box(clip_xy_mm, clip_xy_mm, clip_z_mm)
        .translate((0.0, 0.0, z_plane + clip_z_mm / 2.0))
    )
    return holder_wp.intersect(clip_box)


def calculate_sub_image_lower_left_positions(config: Dict) -> List[Vec2]:
    layout = config["sub_image_layout"]
    sub_w, sub_h = layout["sub_image_size_mm"]
    x_interval = layout["x_interval_mm"]
    y_interval = layout["y_interval_mm"]

    lower_left_positions: List[Vec2] = [
        (x_interval, y_interval),
        (x_interval + sub_w / 4.0, y_interval * 2.0 + sub_h),
        (x_interval + sub_w / 2.0, y_interval * 3.0 + 2.0 * sub_h),
        (x_interval + sub_w * 3.0 / 4.0, y_interval * 4.0 + 3.0 * sub_h),
    ]

    for i in range(1, 7):
        if i == 6:
            new_column = [
                (pos[0] + i * (sub_w + x_interval), pos[1]) for pos in lower_left_positions[:2]
            ]
        else:
            new_column = [
                (pos[0] + i * (sub_w + x_interval), pos[1]) for pos in lower_left_positions[:4]
            ]
        lower_left_positions.extend(new_column)

    return lower_left_positions


def calculate_sub_image_centers(config: Dict) -> List[Vec2]:
    screen_w, screen_h, _ = config["screen"]["size_mm"]
    screen_center = resolve_plane_center(config["screen"])
    sub_w, sub_h = config["sub_image_layout"]["sub_image_size_mm"]
    ll_positions = calculate_sub_image_lower_left_positions(config)

    x_offset = float(screen_center[0]) - screen_w / 2.0
    y_offset = float(screen_center[1]) - screen_h / 2.0
    return [(x_offset + x + sub_w / 2.0, y_offset + y + sub_h / 2.0) for x, y in ll_positions]


def build_screen_and_subimages(config: Dict, sub_centers_2d: List[Vec2]) -> Tuple[cq.Workplane, cq.Workplane | None]:
    screen_cfg = config["screen"]
    screen_size = screen_cfg["size_mm"]
    screen_center = resolve_plane_center(screen_cfg)
    screen_wp = create_box(screen_size, screen_center)

    layout = config["sub_image_layout"]
    emboss_h = float(layout.get("emboss_height_mm", 0.0))
    if emboss_h <= 0.0:
        return screen_wp, None

    sub_w, sub_h = layout["sub_image_size_mm"]
    screen_front_z = resolve_plane_front_z(screen_cfg)
    sub_z = screen_front_z + emboss_h / 2.0

    sub_solids: List[cq.Shape] = []
    for cx, cy in sub_centers_2d:
        sub_solids.append(create_box([sub_w, sub_h, emboss_h], [cx, cy, sub_z]).val())

    return screen_wp, cq.Workplane(obj=cq.Compound.makeCompound(sub_solids))


def calculate_plate_intersections(config: Dict) -> List[Vec3]:
    centers_2d = calculate_sub_image_centers(config)

    z_screen = resolve_plane_front_z(config["screen"])
    z_plate = resolve_plane_front_z(config["holder_array_combo"]["plate"])

    conv = config["convergence_control"]
    z_h = float(conv["horizontal_convergence_y_axis_line_z_mm"])
    z_v = float(conv["vertical_convergence_x_axis_line_z_mm"])

    points: List[Vec3] = []
    for xs, ys in centers_2d:
        x_l = xs + (z_plate - z_screen) / (z_h - z_screen) * (0.0 - xs)
        y_l = ys + (z_plate - z_screen) / (z_v - z_screen) * (0.0 - ys)
        points.append((x_l, y_l, z_plate))

    return points


def build_holder_array_combo(config: Dict, config_path: Path) -> Dict:
    combo_cfg = config.get("holder_array_combo", {})
    if not combo_cfg.get("enabled", True):
        raise ValueError("holder_array_combo.enabled is false")

    plate_cfg = combo_cfg.get("plate", {})
    plate_size = plate_cfg.get("size_mm", config["lens_plate"]["size_mm"])
    if "size_mm" not in plate_cfg:
        plate_cfg = dict(config["lens_plate"])
    plate_center = resolve_plane_center(plate_cfg)
    plate_wp = create_box(plate_size, plate_center)

    holder_template = load_holder_template(config, config_path)
    cutter_cfg = combo_cfg.get("cutter", {})
    cutter_mode = cutter_cfg.get("mode", "max_radius")
    radius_offset = float(cutter_cfg.get("radius_offset_mm", 0.0))
    length_margin = float(cutter_cfg.get("length_margin_mm", 20.0))
    trim_holders = bool(combo_cfg.get("trim_holders_by_plate_back_plane", True))

    template_outer = select_outer_cylinder(get_cylindrical_faces(holder_template.val()), mode=cutter_mode)
    template_axis = normalize_dir(template_outer["axis_dir"])

    sx, sy, sz = plate_size
    cut_length = math.sqrt(sx * sx + sy * sy + sz * sz) + 2.0 * length_margin
    z_back = float(plate_center[2]) - float(plate_size[2]) / 2.0

    z_screen = resolve_plane_front_z(config["screen"])
    sub_centers_2d = calculate_sub_image_centers(config)
    plate_points = calculate_plate_intersections(config)
    screen_wp, subimages_compound = build_screen_and_subimages(config, sub_centers_2d)
    guides_compound = build_visual_guides(config, sub_centers_2d, plate_points)

    holders: List[cq.Workplane] = []
    cut_plate = plate_wp
    radii: List[float] = []

    for (xs, ys), (xl, yl, zl) in zip(sub_centers_2d, plate_points):
        ray_dir = normalize_dir((xl - xs, yl - ys, zl - z_screen))

        holder_i = align_axis_to_target(holder_template, template_axis, ray_dir)
        outer_i = select_outer_cylinder(get_cylindrical_faces(holder_i.val()), mode=cutter_mode)

        # Move holder so its outer-cylinder axis goes through the plate intersection point.
        axis_loc = outer_i["axis_loc"]
        holder_i = holder_i.translate(vector_sub((xl, yl, zl), axis_loc))

        # Re-sample after translation to get final cutter axis.
        outer_i = select_outer_cylinder(get_cylindrical_faces(holder_i.val()), mode=cutter_mode)
        cutter = make_cutter_cylinder(
            axis_loc=outer_i["axis_loc"],
            axis_dir=outer_i["axis_dir"],
            radius=outer_i["radius"] + radius_offset,
            length=cut_length,
        )
        cut_plate = cut_plate.cut(cutter)
        if trim_holders:
            holder_i = trim_holder_keep_front_by_z_plane(holder_i, z_back)
        holders.append(holder_i)
        radii.append(outer_i["radius"])

    holder_shapes = [h.val() for h in holders]
    holders_compound = cq.Workplane(obj=cq.Compound.makeCompound(holder_shapes))

    try:
        combo = cut_plate.union(holders_compound)
    except Exception:
        combo = cq.Workplane(obj=cq.Compound.makeCompound([cut_plate.val(), holders_compound.val()]))

    return {
        "combo": combo,
        "screen": screen_wp,
        "subimages_compound": subimages_compound,
        "guides_compound": guides_compound,
        "holder_count": len(holders),
        "radius_min": min(radii) if radii else 0.0,
        "radius_max": max(radii) if radii else 0.0,
        "z_back_plane_mm": z_back,
        "trim_holders_by_plate_back_plane": trim_holders,
    }


def export_result(result: Dict, output_dir: Path, basename: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    if result.get("guides_compound") is None:
        raise ValueError("visual guides are required for export")

    compound_parts = [result["combo"].val(), result["screen"].val(), result["guides_compound"].val()]
    if result.get("subimages_compound") is not None:
        compound_parts.append(result["subimages_compound"].val())
    with_guides = cq.Workplane(obj=cq.Compound.makeCompound(compound_parts))
    cq.exporters.export(with_guides.val(), str(output_dir / f"{basename}_with_guides.step"))
    cq.exporters.export(with_guides.val(), str(output_dir / f"{basename}_with_guides.stl"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build holder array + cut plate combo")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/system_layout.json"),
        help="Path to JSON config",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = args.config.resolve()
    config = load_config(config_path)

    combo_cfg = config.get("holder_array_combo", {})
    output_cfg = combo_cfg.get("output", {})
    output_dir = Path(output_cfg.get("directory", "output"))
    basename = output_cfg.get("basename", "holder_array_combo")

    result = build_holder_array_combo(config, config_path)
    export_result(result, output_dir, basename)

    print(f"[OK] with-guides STEP: {output_dir / (basename + '_with_guides.step')}")
    print(f"[OK] with-guides STL: {output_dir / (basename + '_with_guides.stl')}")
    print(
        f"[INFO] holder_count={result['holder_count']}, "
        f"radius_range_mm=[{result['radius_min']:.4f}, {result['radius_max']:.4f}]"
    )
    print(
        f"[INFO] trim_holders_by_plate_back_plane={result['trim_holders_by_plate_back_plane']}, "
        f"z_back_plane_mm={result['z_back_plane_mm']:.4f}"
    )


if __name__ == "__main__":
    main()
