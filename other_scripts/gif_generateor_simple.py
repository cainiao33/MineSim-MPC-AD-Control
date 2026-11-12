"""Combine simulator PNG frames into a GIF."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from PIL import Image


def _frame_number(path: Path) -> int:
    try:
        return int(path.stem.rsplit("_", 1)[-1])
    except ValueError:
        return 0


def generate_gif_from_images(dir_pngs, dir_gif, image_scenes_name=None, duration_ms: int = 100):
    """Combine every PNG in ``dir_pngs`` into one naturally sorted GIF."""
    images_dir = Path(dir_pngs).expanduser().resolve()
    output_dir = Path(dir_gif).expanduser().resolve()
    if not images_dir.is_dir():
        raise FileNotFoundError(f"PNG 目录不存在：{images_dir}")
    files = sorted(images_dir.glob("*.png"), key=_frame_number)
    if not files:
        raise ValueError(f"PNG 目录中没有图片：{images_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    gif_name = image_scenes_name or images_dir.parent.name
    if not gif_name.lower().endswith(".gif"):
        gif_name = f"{gif_name}.gif"
    gif_path = output_dir / gif_name
    frames = []
    for path in files:
        with Image.open(path) as image:
            frames.append(image.convert("RGB").copy())
    frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=duration_ms, loop=0)
    return gif_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="把仿真逐帧 PNG 合成为 GIF。")
    parser.add_argument("images_dir", type=Path, help="包含 PNG 的 images 目录")
    parser.add_argument("--output", type=Path, help="输出 GIF 路径，默认放在 images 的上一级目录")
    parser.add_argument("--fps", type=float, default=10.0, help="GIF 帧率")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.fps <= 0:
        parser.error("--fps 必须大于 0")
    images_dir = args.images_dir.expanduser().resolve()
    output = args.output.expanduser().resolve() if args.output else images_dir.parent / f"{images_dir.parent.name}.gif"
    try:
        gif_path = generate_gif_from_images(images_dir, output.parent, output.name, round(1000 / args.fps))
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))
    print(f"###log### GIF 文件：{gif_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
