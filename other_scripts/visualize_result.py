"""Visualize a mine-sim result CSV as a PNG or animated GIF."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Sequence

PROJECT_ROOT = Path(__file__).resolve().parent.parent
os.environ.setdefault("MPLCONFIGDIR", str(PROJECT_ROOT / "outputs" / ".matplotlib"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from PIL import Image


LOCATION_FILES = {
    "dapai": ("guangdong_dapai", "guangdong_dapai_bitmap_mask.png", "guangdong_dapai_semantic_map.json"),
    "jiangtong": ("jiangxi_jiangtong", "jiangxi_jiangtong_bitmap_mask.png", "jiangxi_jiangtong_semantic_map.json"),
}
END_TEXT = {
    -1: "incomplete",
    1: "timeout",
    2: "vehicle collision",
    3: "road-boundary collision",
    4: "goal reached",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="读取仿真结果 CSV，生成轨迹 PNG 或动画 GIF。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("result_csv", type=Path, help="<场景名>_result.csv")
    parser.add_argument("--output", type=Path, help="输出 .png 或 .gif；默认在 CSV 同目录生成 PNG")
    parser.add_argument("--datasets", type=Path, default=PROJECT_ROOT / "datasets", help="用于加载可行驶区域底图的数据集目录")
    parser.add_argument("--no-map", action="store_true", help="不加载地图底图，仅绘制轨迹")
    parser.add_argument("--margin", type=float, default=15.0, help="轨迹边界外额外显示的米数")
    parser.add_argument("--fps", type=float, default=10.0, help="GIF 帧率")
    parser.add_argument("--stride", type=int, default=1, help="GIF 每隔多少行取一帧")
    return parser


def vehicle_ids(frame: pd.DataFrame) -> list[str]:
    ids = [column[2:] for column in frame.columns if column.startswith("x_") and f"y_{column[2:]}" in frame.columns]
    return sorted(ids, key=lambda value: (value != "ego", not value.isdigit(), int(value) if value.isdigit() else value))


def load_result(csv_path: Path) -> pd.DataFrame:
    frame = pd.read_csv(csv_path, index_col=0)
    if frame.empty:
        raise ValueError(f"结果 CSV 没有数据：{csv_path}")
    ids = vehicle_ids(frame)
    if "ego" not in ids:
        raise ValueError("结果 CSV 缺少 x_ego/y_ego 列")
    frame.index = pd.to_numeric(frame.index, errors="coerce")
    return frame


def trajectory_bounds(frame: pd.DataFrame, ids: Sequence[str], margin: float) -> tuple[float, float, float, float]:
    xs = np.concatenate([frame[f"x_{vehicle_id}"].dropna().to_numpy(float) for vehicle_id in ids])
    ys = np.concatenate([frame[f"y_{vehicle_id}"].dropna().to_numpy(float) for vehicle_id in ids])
    return float(xs.min() - margin), float(ys.min() - margin), float(xs.max() + margin), float(ys.max() + margin)


def infer_location(csv_path: Path):
    scene_name = csv_path.stem.removesuffix("_result")
    prefix = scene_name.split("_", 1)[0]
    return LOCATION_FILES.get(prefix)


def load_map_patch(datasets_dir: Path, csv_path: Path, bounds):
    location = infer_location(csv_path)
    if location is None:
        return None
    _, bitmap_name, semantic_name = location
    semantic_path = datasets_dir / "maps" / "semantic_map" / semantic_name
    bitmap_path = datasets_dir / "maps" / "bitmap" / bitmap_name
    if not semantic_path.is_file() or not bitmap_path.is_file():
        return None

    with semantic_path.open("r", encoding="utf-8") as stream:
        bitmap_info = json.load(stream)["bitmap_mask_PNG"]
    x_range = bitmap_info["UTM_info"]["local_x_range"]
    y_range = bitmap_info["UTM_info"]["local_y_range"]
    scale = float(bitmap_info["scale_PixelPerMeter"])

    Image.MAX_IMAGE_PIXELS = None
    with Image.open(bitmap_path) as image:
        width, height = image.size
        x_min = max(bounds[0], float(x_range[0]))
        y_min = max(bounds[1], float(y_range[0]))
        x_max = min(bounds[2], float(x_range[1]))
        y_max = min(bounds[3], float(y_range[1]))
        if x_min >= x_max or y_min >= y_max:
            return None
        left = max(0, int((x_min - x_range[0]) * scale))
        right = min(width, int(np.ceil((x_max - x_range[0]) * scale)))
        top = max(0, height - int(np.ceil((y_max - y_range[0]) * scale)))
        bottom = min(height, height - int((y_min - y_range[0]) * scale))
        patch = np.flipud(np.asarray(image.crop((left, top, right, bottom)).convert("L")))

    extent = (
        x_range[0] + left / scale,
        x_range[0] + right / scale,
        y_range[0] + (height - bottom) / scale,
        y_range[0] + (height - top) / scale,
    )
    return patch, extent


def terminal_status(frame: pd.DataFrame) -> tuple[int, str]:
    value = int(frame["end"].iloc[-1]) if "end" in frame.columns and not pd.isna(frame["end"].iloc[-1]) else -1
    return value, END_TEXT.get(value, "unknown")


def draw_map(ax, map_patch, bounds) -> None:
    if map_patch is not None:
        image, extent = map_patch
        ax.imshow(image, extent=extent, cmap="gray", origin="lower", alpha=0.55, zorder=0)
    ax.set_xlim(bounds[0], bounds[2])
    ax.set_ylim(bounds[1], bounds[3])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.2)


def render_png(frame: pd.DataFrame, csv_path: Path, output_path: Path, map_patch, bounds) -> None:
    ids = vehicle_ids(frame)
    figure, (map_ax, state_ax) = plt.subplots(1, 2, figsize=(14, 7), gridspec_kw={"width_ratios": (3, 2)})
    draw_map(map_ax, map_patch, bounds)
    for vehicle_id in ids:
        label = "ego" if vehicle_id == "ego" else f"vehicle {vehicle_id}"
        width = 2.5 if vehicle_id == "ego" else 1.2
        map_ax.plot(frame[f"x_{vehicle_id}"], frame[f"y_{vehicle_id}"], label=label, linewidth=width)
        valid = frame[[f"x_{vehicle_id}", f"y_{vehicle_id}"]].dropna()
        if not valid.empty:
            map_ax.scatter(valid.iloc[0, 0], valid.iloc[0, 1], marker="o", s=24)
            map_ax.scatter(valid.iloc[-1, 0], valid.iloc[-1, 1], marker="x", s=40)
    end, status = terminal_status(frame)
    map_ax.set_title(f"{csv_path.stem} | end={end} ({status})")
    map_ax.legend(loc="best")

    state_ax.plot(frame.index, frame["v_mps_ego"], label="speed (m/s)")
    state_ax.plot(frame.index, frame["acc_mpss_ego"], label="acceleration (m/s²)")
    state_ax.set_xlabel("simulation time (s)")
    state_ax.set_title("ego state")
    state_ax.grid(alpha=0.25)
    state_ax.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=140)
    plt.close(figure)


def render_gif(frame: pd.DataFrame, csv_path: Path, output_path: Path, map_patch, bounds, fps: float, stride: int) -> None:
    if fps <= 0:
        raise ValueError("--fps 必须大于 0")
    if stride <= 0:
        raise ValueError("--stride 必须大于 0")
    ids = vehicle_ids(frame)
    indices = list(range(0, len(frame), stride))
    if indices[-1] != len(frame) - 1:
        indices.append(len(frame) - 1)
    images = []
    figure, ax = plt.subplots(figsize=(8, 8))
    for row_index in indices:
        ax.clear()
        draw_map(ax, map_patch, bounds)
        for vehicle_id in ids:
            history = frame.iloc[: row_index + 1]
            label = "ego" if vehicle_id == "ego" else f"vehicle {vehicle_id}"
            width = 2.5 if vehicle_id == "ego" else 1.2
            ax.plot(history[f"x_{vehicle_id}"], history[f"y_{vehicle_id}"], label=label, linewidth=width)
            x = frame[f"x_{vehicle_id}"].iloc[row_index]
            y = frame[f"y_{vehicle_id}"].iloc[row_index]
            if not pd.isna(x) and not pd.isna(y):
                ax.scatter(x, y, s=36 if vehicle_id == "ego" else 20)
        ax.set_title(f"{csv_path.stem} | t={frame.index[row_index]:.1f} s")
        ax.legend(loc="best")
        figure.tight_layout()
        figure.canvas.draw()
        images.append(Image.fromarray(np.asarray(figure.canvas.buffer_rgba()).copy()).convert("RGB"))
    plt.close(figure)
    duration_ms = max(1, round(1000 / fps))
    images[0].save(output_path, save_all=True, append_images=images[1:], duration=duration_ms, loop=0)


def visualize(args: argparse.Namespace) -> Path:
    csv_path = args.result_csv.expanduser().resolve()
    if not csv_path.is_file():
        raise FileNotFoundError(f"结果 CSV 不存在：{csv_path}")
    output_path = (args.output or csv_path.with_name(f"{csv_path.stem}_trajectory.png")).expanduser().resolve()
    if output_path.suffix.lower() not in {".png", ".gif"}:
        raise ValueError("--output 必须使用 .png 或 .gif 后缀")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    frame = load_result(csv_path)
    ids = vehicle_ids(frame)
    bounds = trajectory_bounds(frame, ids, args.margin)
    map_patch = None if args.no_map else load_map_patch(args.datasets.expanduser().resolve(), csv_path, bounds)
    if output_path.suffix.lower() == ".gif":
        render_gif(frame, csv_path, output_path, map_patch, bounds, args.fps, args.stride)
    else:
        render_png(frame, csv_path, output_path, map_patch, bounds)
    return output_path


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        output_path = visualize(args)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))
    print(f"###log### 可视化结果：{output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
