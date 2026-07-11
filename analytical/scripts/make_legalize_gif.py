#!/usr/bin/env python3
"""
make_legalize_gif.py
Usage:
    python3 scripts/make_legalize_gif.py <out_dir> [--fps N]

讀取 <out_dir>/tier*/manifest.json（或 fallback 依檔名排序），
以 matplotlib 替每幀加上標題後，用 imageio 合成每層的 GIF。
"""

import argparse
import json
import io
import os
import sys
from pathlib import Path

import imageio.v3 as iio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def frames_for_tier(tier_dir: Path) -> list[tuple[str, str]]:
    """
    回傳 [(檔名, tag), ...] 列表，依 manifest.json 排序；
    若 manifest 不存在則 fallback 依檔名字母排序。
    返回值的 tag 從檔名解析：frame_00001_near->far.png → "near->far"
    """
    manifest = tier_dir / "manifest.json"
    if manifest.exists():
        with open(manifest) as f:
            data = json.load(f)
        fnames = data.get("frames", [])
    else:
        fnames = sorted(p.name for p in tier_dir.glob("frame_*.png"))

    result = []
    for fname in fnames:
        # 解析 tag：去掉 "frame_XXXXX_" 前綴與 ".png" 後綴
        stem = fname.removesuffix(".png")
        parts = stem.split("_", 2)
        tag = parts[2] if len(parts) >= 3 else stem
        result.append((fname, tag))
    return result


def frame_to_rgba(img_path: Path, tag: str, figsize_px: tuple[int, int]) -> np.ndarray:
    """
    用 matplotlib 讀取 PNG，加上標題，回傳 RGBA ndarray（H×W×4）。
    figsize_px：(width, height) in pixels，決定輸出解析度。
    """
    img = iio.imread(img_path)

    dpi = 100
    fw = figsize_px[0] / dpi
    fh = figsize_px[1] / dpi + 0.5  # 額外給標題留空間

    fig, ax = plt.subplots(figsize=(fw, fh), dpi=dpi)
    fig.patch.set_facecolor("#ffffff")
    ax.set_facecolor("#ffffff")
    ax.imshow(img, origin="upper")
    ax.set_title(tag, fontsize=10, pad=4, color="#000000")
    ax.axis("off")
    fig.tight_layout(pad=0.3)

    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return iio.imread(buf)


def make_gif_for_tier(tier_dir: Path, out_gif: Path, fps: int) -> None:
    frame_list = frames_for_tier(tier_dir)
    if not frame_list:
        print(f"  [skip] {tier_dir.name}: no frames found")
        return

    # 取第一張決定輸出尺寸
    first_img = iio.imread(tier_dir / frame_list[0][0])
    h, w = first_img.shape[:2]

    frames_rgba = []
    for fname, tag in frame_list:
        rgba = frame_to_rgba(tier_dir / fname, tag, (w, h))
        frames_rgba.append(rgba)

    duration_ms = int(1000 / fps)
    iio.imwrite(
        str(out_gif),
        frames_rgba,
        extension=".gif",
        duration=duration_ms,
        loop=0,
    )
    print(f"  [{tier_dir.name}] {len(frames_rgba)} frames -> {out_gif}  ({fps} fps)")


def main() -> None:
    parser = argparse.ArgumentParser(description="Legalize 視覺化 GIF 合成")
    parser.add_argument("out_dir", help="legalize_frames 目錄路徑")
    parser.add_argument("--fps", type=int, default=5, help="GIF 播放速率（預設 5）")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    if not out_dir.is_dir():
        print(f"Error: directory not found: {out_dir}", file=sys.stderr)
        sys.exit(1)

    tier_dirs = sorted(
        p for p in out_dir.iterdir()
        if p.is_dir() and p.name.startswith("tier")
    )
    if not tier_dirs:
        print(f"No tier* subdirectories found in {out_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"[LegalizeGIF] {len(tier_dirs)} tier(s) found in {out_dir}")
    for tier_dir in tier_dirs:
        gif_path = out_dir / (tier_dir.name + ".gif")
        make_gif_for_tier(tier_dir, gif_path, args.fps)

    print("[LegalizeGIF] Done.")


if __name__ == "__main__":
    main()
