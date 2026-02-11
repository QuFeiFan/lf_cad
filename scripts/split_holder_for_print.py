#!/usr/bin/env python3
"""Split holder assembly into two printable parts with one polyline vertical cut.

The split path is auto-generated to avoid crossing circular holder cutouts.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Tuple

import cadquery as cq

# Reuse geometry construction from current main modeling script.
import importlib.util

Vec2 = Tuple[float, float]


def load_builder_module(path: Path):
    spec = importlib.util.spec_from_file_location("holder_builder", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def point_to_segment_distance_2d(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    vv = vx * vx + vy * vy
    if vv <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = (wx * vx + wy * vy) / vv
    t = max(0.0, min(1.0, t))
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


def min_distance_to_polyline_2d(point_xy: Vec2, polyline_xy: List[Vec2]) -> float:
    px, py = point_xy
    if len(polyline_xy) < 2:
        raise ValueError("polyline requires at least 2 points")
    d_min = float("inf")
    for i in range(len(polyline_xy) - 1):
        ax, ay = polyline_xy[i]
        bx, by = polyline_xy[i + 1]
        d = point_to_segment_distance_2d(px, py, ax, ay, bx, by)
        if d < d_min:
            d_min = d
    return d_min


def make_split_prisms_from_polyline(
    polyline_xy: List[Vec2],
    extent_xy: float = 5000.0,
    extent_z: float = 5000.0,
) -> Tuple[cq.Workplane, cq.Workplane]:
    if len(polyline_xy) < 2:
        raise ValueError("polyline requires at least 2 points")

    x_lo = -extent_xy / 2.0
    x_hi = extent_xy / 2.0
    y_lo = float(polyline_xy[0][1])
    y_hi = float(polyline_xy[-1][1])
    if y_hi <= y_lo:
        raise ValueError("polyline y coordinates must be increasing from start to end")

    left_polygon = [(x_lo, y_lo), (x_lo, y_hi)] + list(reversed(polyline_xy))
    right_polygon = list(polyline_xy) + [(x_hi, y_hi), (x_hi, y_lo)]

    left_prism = (
        cq.Workplane("XY")
        .polyline(left_polygon)
        .close()
        .extrude(extent_z)
        .translate((0.0, 0.0, -extent_z / 2.0))
    )
    right_prism = (
        cq.Workplane("XY")
        .polyline(right_polygon)
        .close()
        .extrude(extent_z)
        .translate((0.0, 0.0, -extent_z / 2.0))
    )
    return left_prism, right_prism


def extract_split_circles(mod, cfg: Dict, cfg_path: Path, avoid_clearance_mm: float) -> List[Dict]:
    plate_points = mod.calculate_plate_intersections(cfg)
    holder_template = mod.load_holder_template(cfg, cfg_path)
    cutter_mode = cfg.get("holder_array_combo", {}).get("cutter", {}).get("mode", "max_radius")
    outer = mod.select_outer_cylinder(mod.get_cylindrical_faces(holder_template.val()), mode=cutter_mode)
    radius = float(outer["radius"])

    circles = []
    for x, y, _ in plate_points:
        circles.append(
            {
                "x": float(x),
                "y": float(y),
                "radius": radius,
                "inflated_radius": radius + avoid_clearance_mm,
            }
        )
    return circles


def group_circles_by_row(circles: List[Dict], y_round_digits: int = 3) -> List[Dict]:
    rows: Dict[float, List[Dict]] = {}
    for c in circles:
        key = round(float(c["y"]), y_round_digits)
        rows.setdefault(key, []).append(c)

    row_list = []
    for key in sorted(rows):
        row_circles = sorted(rows[key], key=lambda v: float(v["x"]))
        y_avg = sum(float(c["y"]) for c in row_circles) / len(row_circles)
        row_list.append({"y": y_avg, "circles": row_circles})
    return row_list


def choose_row_split_x(rows: List[Dict], target_x_mm: float = 0.0) -> List[Dict]:
    if not rows:
        raise ValueError("No circle rows found for split planning")

    out = []
    for row in rows:
        y = float(row["y"])
        circles = row["circles"]
        if len(circles) < 2:
            raise ValueError("Each row requires at least 2 circles to find safe corridor")

        candidates = []
        for left, right in zip(circles, circles[1:]):
            lo = float(left["x"]) + float(left["inflated_radius"])
            hi = float(right["x"]) - float(right["inflated_radius"])
            if hi > lo:
                mid = (lo + hi) / 2.0
                width = hi - lo
                candidates.append({"lo": lo, "hi": hi, "mid": mid, "width": width})

        if not candidates:
            raise ValueError(f"No valid split corridor in row y={y:.3f}")

        # Choose corridor whose midpoint is closest to center (target_x_mm).
        best = min(candidates, key=lambda c: (abs(c["mid"] - target_x_mm), -c["width"]))
        out.append({"y": y, "x": best["mid"], "lo": best["lo"], "hi": best["hi"]})

    return sorted(out, key=lambda r: float(r["y"]))


def build_zigzag_polyline(row_split_points: List[Dict], y_extent_mm: float = 2500.0) -> List[Vec2]:
    if not row_split_points:
        raise ValueError("row_split_points is empty")

    polyline: List[Vec2] = []
    first_x = float(row_split_points[0]["x"])
    polyline.append((first_x, -y_extent_mm))

    for i, row in enumerate(row_split_points):
        x_i = float(row["x"])
        y_i = float(row["y"])
        if i == len(row_split_points) - 1:
            polyline.append((x_i, y_extent_mm))
            break

        y_next = float(row_split_points[i + 1]["y"])
        x_next = float(row_split_points[i + 1]["x"])
        y_mid = 0.5 * (y_i + y_next)
        polyline.append((x_i, y_mid))
        polyline.append((x_next, y_mid))

    # Drop accidental consecutive duplicates.
    compact: List[Vec2] = [polyline[0]]
    for p in polyline[1:]:
        if abs(p[0] - compact[-1][0]) > 1e-9 or abs(p[1] - compact[-1][1]) > 1e-9:
            compact.append(p)
    return compact


def evaluate_polyline_clearance(circles: List[Dict], polyline_xy: List[Vec2]) -> Dict:
    min_dist = float("inf")
    min_clearance = float("inf")
    worst = None
    for idx, c in enumerate(circles, 1):
        x = float(c["x"])
        y = float(c["y"])
        r = float(c["radius"])
        d = min_distance_to_polyline_2d((x, y), polyline_xy)
        clr = d - r
        if d < min_dist:
            min_dist = d
            min_clearance = clr
            worst = {"index": idx, "x": x, "y": y, "distance": d, "clearance": clr}
    return {"min_distance": min_dist, "min_clearance": min_clearance, "worst": worst}


def resolve_split_settings(cfg: Dict, args: argparse.Namespace) -> Dict:
    split_cfg = cfg.get("holder_array_combo", {}).get("print_split", {})
    avoid_clearance_mm = (
        float(args.avoid_clearance_mm)
        if args.avoid_clearance_mm is not None
        else float(split_cfg.get("avoid_clearance_mm", 2.0))
    )
    max_part_dim_mm = (
        float(args.max_part_dim_mm)
        if args.max_part_dim_mm is not None
        else float(split_cfg.get("max_part_dim_mm", 240.0))
    )
    return {
        "avoid_clearance_mm": avoid_clearance_mm,
        "max_part_dim_mm": max_part_dim_mm,
    }


def largest_solid_wp(wp: cq.Workplane) -> cq.Workplane:
    solids = list(wp.val().Solids())
    if not solids:
        raise ValueError("Split produced empty part")
    largest = max(solids, key=lambda s: s.Volume())
    return cq.Workplane(obj=largest)


def get_bbox_dims(wp: cq.Workplane) -> Tuple[float, float, float]:
    bb = wp.val().BoundingBox()
    return float(bb.xlen), float(bb.ylen), float(bb.zlen)


def print_bbox(tag: str, dims: Tuple[float, float, float]):
    dx, dy, dz = dims
    print(
        f"[INFO] {tag} bbox_mm=({dx:.3f}, {dy:.3f}, {dz:.3f}) "
        f"max_dim_mm={max(dx, dy, dz):.3f}"
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Split holder assembly for printer bed constraints")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/system_layout.json"),
        help="Path to system config",
    )
    parser.add_argument(
        "--builder",
        type=Path,
        default=Path("scripts/build_holder_array_combo.py"),
        help="Path to holder assembly builder script",
    )
    parser.add_argument(
        "--avoid-clearance-mm",
        type=float,
        default=None,
        help="Override extra XY clearance vs circle radius; default from JSON print_split.avoid_clearance_mm",
    )
    parser.add_argument(
        "--max-part-dim-mm",
        type=float,
        default=None,
        help="Override max allowed dimension for each split part; default from JSON print_split.max_part_dim_mm",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    cfg_path = args.config.resolve()
    builder_path = args.builder.resolve()

    mod = load_builder_module(builder_path)
    cfg = mod.load_config(cfg_path)
    split_settings = resolve_split_settings(cfg, args)
    result = mod.build_holder_array_combo(cfg, cfg_path)

    combo = result["combo"]

    circles = extract_split_circles(
        mod,
        cfg,
        cfg_path,
        avoid_clearance_mm=split_settings["avoid_clearance_mm"],
    )
    rows = group_circles_by_row(circles)
    row_split_points = choose_row_split_x(rows, target_x_mm=0.0)
    split_polyline_xy = build_zigzag_polyline(row_split_points, y_extent_mm=2500.0)

    clearance = evaluate_polyline_clearance(circles, split_polyline_xy)
    if clearance["min_clearance"] <= 0.0:
        w = clearance["worst"]
        raise ValueError(
            "Split polyline intersects a protected circle: "
            f"idx={w['index']}, center=({w['x']:.3f},{w['y']:.3f}), "
            f"clearance_mm={w['clearance']:.6f}"
        )

    left_half, right_half = make_split_prisms_from_polyline(
        split_polyline_xy,
        extent_xy=5000.0,
        extent_z=5000.0,
    )

    part_a = largest_solid_wp(combo.intersect(left_half))
    part_b = largest_solid_wp(combo.intersect(right_half))
    dims_a = get_bbox_dims(part_a)
    dims_b = get_bbox_dims(part_b)
    max_part_dim_limit = split_settings["max_part_dim_mm"]
    max_a = max(dims_a)
    max_b = max(dims_b)
    if max_a > max_part_dim_limit + 1e-9 or max_b > max_part_dim_limit + 1e-9:
        raise ValueError(
            "Split exceeds configured max part dimension: "
            f"limit_mm={max_part_dim_limit:.3f}, part_A_max_mm={max_a:.3f}, part_B_max_mm={max_b:.3f}"
        )

    out_dir = Path(cfg["holder_array_combo"]["output"]["directory"]) / "print_split"
    out_dir.mkdir(parents=True, exist_ok=True)

    step_a = out_dir / "holder_part_A.step"
    stl_a = out_dir / "holder_part_A.stl"
    step_b = out_dir / "holder_part_B.step"
    stl_b = out_dir / "holder_part_B.stl"

    cq.exporters.export(part_a.val(), str(step_a))
    cq.exporters.export(part_a.val(), str(stl_a))
    cq.exporters.export(part_b.val(), str(step_b))
    cq.exporters.export(part_b.val(), str(stl_b))

    print(f"[OK] {step_a}")
    print(f"[OK] {stl_a}")
    print(f"[OK] {step_b}")
    print(f"[OK] {stl_b}")
    print(
        f"[INFO] split_polyline_circle_clearance_mm="
        f"{clearance['min_clearance']:.3f} (avoid_clearance_mm={split_settings['avoid_clearance_mm']:.3f})"
    )
    print("[INFO] split_polyline_xy_points=" + ", ".join(f"({x:.3f},{y:.3f})" for x, y in split_polyline_xy))
    print(
        f"[INFO] split_size_limit_check=PASS "
        f"(max_part_dim_limit_mm={max_part_dim_limit:.3f}, "
        f"part_A_max_mm={max_a:.3f}, part_B_max_mm={max_b:.3f})"
    )
    print_bbox("part_A", dims_a)
    print_bbox("part_B", dims_b)


if __name__ == "__main__":
    main()
