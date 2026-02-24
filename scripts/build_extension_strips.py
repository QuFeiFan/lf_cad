#!/usr/bin/env python3
"""为已打印的板件生成补条（沿 Y 方向补高）。

典型场景：
- 已有板件：404 x 217
- 目标板件：404 x 237
- 需要上下各补一条，单条宽度 = (目标高度 - 现有高度) / 2
- 每条可沿长度方向再切分为 N 份（默认 N=2）

输出为单个 STL 文件，内部包含所有小件并完成排版。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cadquery as cq


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="生成补条小件（单 STL 文件）")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/system_layout.json"),
        help="配置文件路径（JSON）",
    )
    parser.add_argument(
        "--base-height-mm",
        type=float,
        default=217.0,
        help="已打印板件在 Y 方向高度（mm）",
    )
    parser.add_argument(
        "--split-count-length",
        type=int,
        default=2,
        help="每条补条沿 X 方向切分份数",
    )
    parser.add_argument(
        "--gap-mm",
        type=float,
        default=5.0,
        help="输出排版时小件间距（mm）",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/print_split/extension_strips_for_404x237_from_217.stl"),
        help="输出 STL 路径（单文件包含全部小件）",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.split_count_length <= 0:
        raise ValueError("split-count-length 必须 >= 1")

    cfg = load_config(args.config.resolve())
    plate_size = cfg["holder_array_combo"]["plate"]["size_mm"]
    plate_w = float(plate_size[0])
    target_h = float(plate_size[1])
    thickness = float(plate_size[2])
    base_h = float(args.base_height_mm)

    delta_h = target_h - base_h
    if delta_h <= 0.0:
        raise ValueError(
            f"target_height ({target_h:.3f}) 必须大于 base_height ({base_h:.3f})"
        )

    strip_w = delta_h / 2.0
    piece_len = plate_w / float(args.split_count_length)

    # 生成上下两条补条，每条再切分为 N 份。
    # 将全部小件排成一列，方便切片。
    pieces = []
    y_cursor = 0.0
    for strip_idx in range(2):
        for part_idx in range(args.split_count_length):
            solid = (
                cq.Workplane("XY")
                .box(piece_len, strip_w, thickness)
                .translate((0.0, y_cursor, thickness / 2.0))
            )
            pieces.append(solid.val())
            y_cursor += strip_w + float(args.gap_mm)
        # 上下两条补条之间增加额外间距。
        if strip_idx == 0:
            y_cursor += float(args.gap_mm)

    compound = cq.Compound.makeCompound(pieces)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(compound, str(args.output))

    print(f"[OK] {args.output}")
    print(
        "[INFO] piece_count={}, each_piece_mm=({:.3f}, {:.3f}, {:.3f}), "
        "target_plate_mm=({:.3f}, {:.3f}, {:.3f}), base_height_mm={:.3f}".format(
            len(pieces),
            piece_len,
            strip_w,
            thickness,
            plate_w,
            target_h,
            thickness,
            base_h,
        )
    )


if __name__ == "__main__":
    main()
