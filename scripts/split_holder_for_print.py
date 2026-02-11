#!/usr/bin/env python3
"""Split holder assembly into two printable parts with one oblique vertical cut."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cadquery as cq

# Reuse geometry construction from current main modeling script.
import importlib.util


def load_builder_module(path: Path):
    spec = importlib.util.spec_from_file_location("holder_builder", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def make_halfspace_box(normal_xy, offset, keep_positive, size_xy=5000.0, size_z=5000.0):
    nx, ny = normal_xy
    phi_deg = math.degrees(math.atan2(ny, nx))

    x_center = size_xy / 2.0 if keep_positive else -size_xy / 2.0
    halfspace = cq.Workplane("XY").box(size_xy, size_xy, size_z).translate((x_center, 0.0, 0.0))
    halfspace = halfspace.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), phi_deg)

    p0x = nx * offset
    p0y = ny * offset
    halfspace = halfspace.translate((p0x, p0y, 0.0))
    return halfspace


def largest_solid_wp(wp: cq.Workplane) -> cq.Workplane:
    solids = list(wp.val().Solids())
    if not solids:
        raise ValueError("Split produced empty part")
    largest = max(solids, key=lambda s: s.Volume())
    return cq.Workplane(obj=largest)


def print_bbox(tag: str, wp: cq.Workplane):
    bb = wp.val().BoundingBox()
    print(
        f"[INFO] {tag} bbox_mm=({bb.xlen:.3f}, {bb.ylen:.3f}, {bb.zlen:.3f}) "
        f"max_dim_mm={max(bb.xlen, bb.ylen, bb.zlen):.3f}"
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Split holder assembly for 240mm printer")
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
    return parser.parse_args()


def main():
    args = parse_args()
    cfg_path = args.config.resolve()
    builder_path = args.builder.resolve()

    mod = load_builder_module(builder_path)
    cfg = mod.load_config(cfg_path)
    result = mod.build_holder_array_combo(cfg, cfg_path)

    combo = result["combo"]

    # Validated one-cut solution:
    # n dot [x,y] = t with n=(cos(166.4deg), sin(166.4deg)), t=13.3669
    # equivalent line: x ~= 0.2419*y - 13.751 (mm)
    normal_xy = (-0.9719610005785463, 0.23514211310259003)
    offset = 13.366906707792907

    left_half = make_halfspace_box(normal_xy, offset, keep_positive=True)
    right_half = make_halfspace_box(normal_xy, offset, keep_positive=False)

    part_a = largest_solid_wp(combo.intersect(left_half))
    part_b = largest_solid_wp(combo.intersect(right_half))

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
    print_bbox("part_A", part_a)
    print_bbox("part_B", part_b)


if __name__ == "__main__":
    main()
