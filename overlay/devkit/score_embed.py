"""嵌入式真评分器（Route A 核心）：对假想轨迹复刻官方评分。

【vendored 10-02：msim_latff_lab/tmp_scripts/score_embed.py → devkit/（SAS v2
在环评分用）。目录结构同构（devkit/../other_scripts 与 tmp_scripts/../other_scripts
等价），评分代码逐字节未改；新增 install_map_cache()=地图补丁按 (location,
datasets_dir) 进程内缓存（load_map_patch 1.8s/次→一次性），判分数学零接触。】
校验血统：40/40 场与官方 score.json 逐位一致（validate_embed.py）。

复用 other_scripts/score_result.py 的判分函数（traffic_metrics /
lateral_acceleration / violation_duration / road_boundary_collision），
仅把"从结果 CSV 读 DataFrame"换成"从假想轨迹数组构造 DataFrame"——
评分逻辑零改写，保证与官方逐位一致。

背景车来源两种：
  a) 从真结果 CSV 拷列（校验模式：应与官方 score.json 逐位相等）
  b) 从场景 JSON 的 vehicle_traj 按绝对时间键采样（候选评估模式：
     回放真值 = 仿真实际播出的背景车，两者本就相同）

用法（校验）：
  SIM/tmp_scripts/validate_embed.py  # 对 e2_final 40 场逐位 diff
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

LAB_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(LAB_ROOT / "other_scripts"))

from score_result import (  # noqa: E402
    DEFAULT_LATERAL_ACCEL_THRESHOLD_MPSS,
    DEFAULT_LATERAL_DISTANCE_THRESHOLD_M,
    DEFAULT_TTC_THRESHOLD_S,
    lateral_acceleration,
    road_boundary_collision,
    traffic_metrics,
    violation_duration,
)

def build_frame(
    times: np.ndarray,
    ego: dict,
    background: dict[str, dict] | None = None,
    terminal: int = 4,
    ego_shape: dict | None = None,
) -> pd.DataFrame:
    """构造 score_result 兼容 DataFrame。

    ego: {x,y,yaw,v} 各为 len(times) 数组
    background: {vid: {x,y,yaw,v,width,head,rear}}（缺帧→NaN 列，traffic_metrics 自跳过）
    terminal: 末帧 end 值（4=到达）
    ego_shape: {width,length,head,rear}——必须用场景真实值（定位点前后
        因车型而异，NTE200=6.5/2.5，XG90G=2.5/6.5；错一个=足迹移 4m=压界误判）
    """
    shape = dict(width=6.7, length=13.4, head=6.5, rear=2.5)
    shape.update(ego_shape or {})
    n = len(times)
    data = {
        "x_ego": ego["x"],
        "y_ego": ego["y"],
        "yaw_rad_ego": ego["yaw"],
        "v_mps_ego": ego["v"],
        "acc_mpss_ego": np.gradient(ego["v"], times),
        "width_ego": np.full(n, shape["width"]),
        "length_ego": np.full(n, shape["length"]),
        "locationPoint2Head_ego": np.full(n, shape["head"]),
        "locationPoint2Rear_ego": np.full(n, shape["rear"]),
        "end": np.full(n, -1.0),
    }
    data["end"][-1] = float(terminal)
    for vid, tr in (background or {}).items():
        for key, col in (("x", "x"), ("y", "y"), ("yaw", "yaw_rad"), ("v", "v_mps")):
            data[f"{col}_{vid}"] = tr[key]
        data[f"width_{vid}"] = np.full(n, tr.get("width", 5.0))
        data[f"locationPoint2Head_{vid}"] = np.full(n, tr.get("head", 3.0))
        data[f"locationPoint2Rear_{vid}"] = np.full(n, tr.get("rear", 3.0))
    return pd.DataFrame(data, index=pd.Index(times, name="time_s"))


def score_frame(
    frame: pd.DataFrame,
    scene_name: str,
    datasets_dir: Path,
    expected_time: float,
    ttc_threshold: float = DEFAULT_TTC_THRESHOLD_S,
    lateral_distance_threshold: float = DEFAULT_LATERAL_DISTANCE_THRESHOLD_M,
    lateral_acc_threshold: float = DEFAULT_LATERAL_ACCEL_THRESHOLD_MPSS,
) -> dict:
    """对构造好的 frame 评分（与 score_csv 同口径；csv_path 仅用于地图名推断）。"""
    times = frame.index.to_numpy(float)
    actual_time = float(times[-1] - times[0])
    traffic = traffic_metrics(frame, ttc_threshold, lateral_distance_threshold)
    lateral_acc = lateral_acceleration(frame)
    # road_boundary_collision 只用 stem 推断地图，不读文件
    fake_csv = Path(f"{scene_name}_result.csv")
    road_collision, road_check = road_boundary_collision(frame, fake_csv, datasets_dir)
    terminal = int(frame["end"].iloc[-1])
    vehicle_collision = traffic["vehicle_collision"] or terminal == 2

    _, ttc_ratio = violation_duration(traffic["ttc_violation"], times)
    _, lateral_ratio = violation_duration(traffic["lateral_violation"], times)
    _, alat_ratio = violation_duration(lateral_acc > lateral_acc_threshold, times)
    components = {
        "ttc": 20.0 * (1.0 - ttc_ratio),
        "lateral_distance": 20.0 * (1.0 - lateral_ratio),
        "lateral_acceleration": 20.0 * (1.0 - alat_ratio),
    }
    safety = 0.0 if vehicle_collision or road_collision else sum(components.values())
    completion = 20.0 if terminal == 4 else 0.0
    time_score = 20.0 * min(1.0, expected_time / actual_time) if actual_time > 0 else 0.0
    return {
        "safety": safety,
        "efficiency": completion + time_score,
        "total": safety + completion + time_score,
        "components": components,
        "ratios": {"ttc": ttc_ratio, "lat": lateral_ratio, "alat": alat_ratio},
        "vehicle_collision": bool(vehicle_collision),
        "road_boundary_collision": bool(road_collision),
        "road_check": road_check,
        "terminal": terminal,
        "actual_time": actual_time,
    }


def frame_from_result_csv(csv_path: Path) -> pd.DataFrame:
    """校验模式：直接读官方结果 CSV 为 frame（load_result 同款）。"""
    frame = pd.read_csv(csv_path, index_col=0)
    frame.index = pd.to_numeric(frame.index, errors="coerce")
    return frame


# ---------------- 地图补丁进程内缓存 v2（D3；判分数学零接触） ----------------
# v1 按实参 repr 键缓存全 miss——road_boundary_collision 的 crop 界=逐候选
# ego 轨迹 bounds（浮点逐候选必异）。v2 缓存下移到 I/O 层：
#   · 语义 json 按 (datasets, semantic) 一次解析；
#   · 全图 flipud(convert("L")) 数组按 (datasets, bitmap) 一次解码
#     （dapai 273MB / jiangtong 445MB；fork 子进程 COW 只读共享）；
#   · patch=缓存数组的零拷贝视图——slice-of-flipud ≡ flipud-of-slice
#     （crop/convert 均逐像素矩形运算，可交换）；int/ceil 裁剪算术与
#     visualize_result.load_map_patch 逐行同码。
# 不复用 BitMap 的 convert("1") 数组：PIL L→1 默认 Floyd-Steinberg 抖动，
# 二值图上恰与评分器 >=128 等价但不可证明（灰边像素会分歧）。
# 离线等价：两图 ×13 组 bounds（随机+越界/退化/贴边）patch 像素+extent EXACT。

_MAP_INFO: dict = {}   # (datasets, semantic_name) -> bitmap_info dict
_MAP_FULL: dict = {}   # (datasets, bitmap_name) -> (flipped_L ndarray, w, h)


def install_map_cache() -> None:
    """把 score_result.load_map_patch 换成全量缓存版（幂等；判分数学零接触）。"""
    import json

    import score_result as _sr

    if getattr(_sr, "_sas_map_cached", False):
        return

    def _cached_load_map_patch(datasets_dir, csv_path, bounds):
        from visualize_result import infer_location

        location = infer_location(csv_path)
        if location is None:
            return None
        _, bitmap_name, semantic_name = location
        datasets_dir = Path(datasets_dir)
        ik = (str(datasets_dir), semantic_name)
        fk = (str(datasets_dir), bitmap_name)
        if fk not in _MAP_FULL:
            semantic_path = datasets_dir / "maps" / "semantic_map" / semantic_name
            bitmap_path = datasets_dir / "maps" / "bitmap" / bitmap_name
            if not semantic_path.is_file() or not bitmap_path.is_file():
                return None
            with semantic_path.open("r", encoding="utf-8") as stream:
                _MAP_INFO[ik] = json.load(stream)["bitmap_mask_PNG"]
            from PIL import Image

            Image.MAX_IMAGE_PIXELS = None
            with Image.open(bitmap_path) as image:
                width, height = image.size
                _MAP_FULL[fk] = (
                    np.flipud(np.asarray(image.convert("L"))), width, height)
        bitmap_info = _MAP_INFO[ik]
        full, width, height = _MAP_FULL[fk]
        # ↓ visualize_result.load_map_patch 算术逐行同码（bounds→像素盒）
        x_range = bitmap_info["UTM_info"]["local_x_range"]
        y_range = bitmap_info["UTM_info"]["local_y_range"]
        scale = float(bitmap_info["scale_PixelPerMeter"])
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
        patch = full[height - bottom:height - top, left:right]  # 零拷贝视图
        extent = (
            x_range[0] + left / scale,
            x_range[0] + right / scale,
            y_range[0] + (height - bottom) / scale,
            y_range[0] + (height - top) / scale,
        )
        return patch, extent

    _sr.load_map_patch = _cached_load_map_patch
    _sr._sas_map_cached = True
