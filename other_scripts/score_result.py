"""Score mine-sim result CSV files using the documented 100-point rule."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Sequence

import numpy as np
import pandas as pd
from PIL import Image, ImageDraw
from shapely.geometry import Polygon

from visualize_result import load_map_patch, load_result, trajectory_bounds, vehicle_ids


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_EXPECTED_TIMES = PROJECT_ROOT / "inputs" / "expected_times.json"
DEFAULT_TTC_THRESHOLD_S = 3.0
DEFAULT_LATERAL_DISTANCE_THRESHOLD_M = 0.8
DEFAULT_LATERAL_ACCEL_THRESHOLD_MPSS = 0.4


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="按照赛题说明的安全 60 分、效率 40 分规则评价结果 CSV。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("results", nargs="+", type=Path, help="结果 CSV，或包含 *_result.csv 的目录")
    expected = parser.add_mutually_exclusive_group()
    expected.add_argument("--expected-time", type=float, help="期望耗时（秒）；应用于本次输入的全部场景")
    expected.add_argument(
        "--expected-times",
        type=Path,
        help="场景名到期望耗时的 JSON 对象；默认读取 inputs/expected_times.json",
    )
    parser.add_argument("--datasets", type=Path, default=PROJECT_ROOT / "datasets", help="道路边界复核使用的数据集目录")
    parser.add_argument("--output", type=Path, help="评分 JSON；单场景默认与结果 CSV 同目录，多场景默认当前目录")
    parser.add_argument("--ttc-threshold", type=float, default=DEFAULT_TTC_THRESHOLD_S, help="前车 TTC 安全阈值（秒）")
    parser.add_argument(
        "--lateral-distance-threshold",
        type=float,
        default=DEFAULT_LATERAL_DISTANCE_THRESHOLD_M,
        help="侧向车辆外轮廓安全距离（米）",
    )
    parser.add_argument(
        "--lateral-acc-threshold",
        type=float,
        default=DEFAULT_LATERAL_ACCEL_THRESHOLD_MPSS,
        help="侧向加速度绝对值阈值（m/s²）",
    )
    return parser


def _finite_number(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def validate_result(frame: pd.DataFrame) -> list[str]:
    errors = []
    required = {
        "x_ego",
        "y_ego",
        "yaw_rad_ego",
        "v_mps_ego",
        "acc_mpss_ego",
        "width_ego",
        "length_ego",
        "locationPoint2Head_ego",
        "locationPoint2Rear_ego",
        "end",
    }
    missing = sorted(required - set(frame.columns))
    if missing:
        errors.append(f"缺少列：{', '.join(missing)}")
        return errors
    if len(frame) < 2:
        errors.append("至少需要两个时间采样点")
    times = frame.index.to_numpy(float)
    if not np.all(np.isfinite(times)) or len(times) > 1 and not np.all(np.diff(times) > 0):
        errors.append("时间索引必须是严格递增的有限数值")
    if frame[list(required - {"end"})].isna().any().any():
        errors.append("自车数据存在空值")
    terminal = frame["end"].iloc[-1]
    if not _finite_number(terminal) or int(terminal) not in {1, 2, 3, 4}:
        errors.append("末帧没有有效终止状态，视为算法失效")
    return errors


def _vehicle_columns_available(row: pd.Series, vehicle_id: str) -> bool:
    columns = (
        f"x_{vehicle_id}",
        f"y_{vehicle_id}",
        f"yaw_rad_{vehicle_id}",
        f"v_mps_{vehicle_id}",
        f"width_{vehicle_id}",
        f"locationPoint2Head_{vehicle_id}",
        f"locationPoint2Rear_{vehicle_id}",
    )
    return all(column in row.index and _finite_number(row[column]) for column in columns)


def vehicle_corners(row: pd.Series, vehicle_id: str) -> np.ndarray:
    x = float(row[f"x_{vehicle_id}"])
    y = float(row[f"y_{vehicle_id}"])
    yaw = float(row[f"yaw_rad_{vehicle_id}"])
    width = float(row[f"width_{vehicle_id}"])
    head = float(row[f"locationPoint2Head_{vehicle_id}"])
    rear = float(row[f"locationPoint2Rear_{vehicle_id}"])
    direction = np.array((math.cos(yaw), math.sin(yaw)))
    lateral = np.array((-math.sin(yaw), math.cos(yaw)))
    position = np.array((x, y))
    front = position + head * direction
    back = position - rear * direction
    half_width = width / 2 * lateral
    return np.array((front + half_width, front - half_width, back - half_width, back + half_width))


def vehicle_polygon(row: pd.Series, vehicle_id: str) -> Polygon:
    return Polygon(vehicle_corners(row, vehicle_id)).convex_hull


def _projection_interval(points: np.ndarray, axis: np.ndarray) -> tuple[float, float]:
    values = points @ axis
    return float(values.min()), float(values.max())


def traffic_metrics(frame: pd.DataFrame, ttc_threshold: float, lateral_distance_threshold: float):
    ids = [vehicle_id for vehicle_id in vehicle_ids(frame) if vehicle_id != "ego"]
    ttc_violation = np.zeros(len(frame), dtype=bool)
    lateral_violation = np.zeros(len(frame), dtype=bool)
    vehicle_collision = False
    minimum_ttc = math.inf
    minimum_lateral_distance = math.inf

    for row_index, (_, row) in enumerate(frame.iterrows()):
        ego_corners = vehicle_corners(row, "ego")
        ego_polygon = Polygon(ego_corners).convex_hull
        ego_yaw = float(row["yaw_rad_ego"])
        forward = np.array((math.cos(ego_yaw), math.sin(ego_yaw)))
        lateral_axis = np.array((-math.sin(ego_yaw), math.cos(ego_yaw)))
        ego_longitudinal = _projection_interval(ego_corners, forward)
        ego_lateral = _projection_interval(ego_corners, lateral_axis)
        ego_speed = float(row["v_mps_ego"])

        for vehicle_id in ids:
            if not _vehicle_columns_available(row, vehicle_id):
                continue
            other_corners = vehicle_corners(row, vehicle_id)
            other_polygon = Polygon(other_corners).convex_hull
            if ego_polygon.intersects(other_polygon):
                vehicle_collision = True

            other_longitudinal = _projection_interval(other_corners, forward)
            other_lateral = _projection_interval(other_corners, lateral_axis)
            longitudinal_overlap = min(ego_longitudinal[1], other_longitudinal[1]) - max(
                ego_longitudinal[0], other_longitudinal[0]
            )
            lateral_overlap = min(ego_lateral[1], other_lateral[1]) - max(ego_lateral[0], other_lateral[0])

            # A front vehicle overlaps the ego's lateral corridor and is ahead
            # of the ego front bumper. TTC uses the longitudinal bumper gap.
            gap = other_longitudinal[0] - ego_longitudinal[1]
            if gap >= 0 and lateral_overlap >= 0:
                other_yaw = float(row[f"yaw_rad_{vehicle_id}"])
                other_velocity = float(row[f"v_mps_{vehicle_id}"]) * np.array((math.cos(other_yaw), math.sin(other_yaw)))
                closing_speed = ego_speed - float(other_velocity @ forward)
                if closing_speed > 1e-6:
                    ttc = gap / closing_speed
                    minimum_ttc = min(minimum_ttc, ttc)
                    if ttc < ttc_threshold:
                        ttc_violation[row_index] = True

            # A lateral vehicle overlaps the ego's longitudinal footprint.
            if longitudinal_overlap >= 0:
                distance = float(ego_polygon.distance(other_polygon))
                minimum_lateral_distance = min(minimum_lateral_distance, distance)
                if distance < lateral_distance_threshold:
                    lateral_violation[row_index] = True

    return {
        "vehicle_collision": vehicle_collision,
        "ttc_violation": ttc_violation,
        "lateral_violation": lateral_violation,
        "minimum_ttc_s": None if math.isinf(minimum_ttc) else minimum_ttc,
        "minimum_lateral_distance_m": None if math.isinf(minimum_lateral_distance) else minimum_lateral_distance,
    }


def lateral_acceleration(frame: pd.DataFrame) -> np.ndarray:
    times = frame.index.to_numpy(float)
    yaw = np.unwrap(frame["yaw_rad_ego"].to_numpy(float))
    yaw_rate = np.gradient(yaw, times)
    return np.abs(frame["v_mps_ego"].to_numpy(float) * yaw_rate)


def violation_duration(mask: np.ndarray, times: np.ndarray) -> tuple[float, float]:
    duration = float(times[-1] - times[0])
    if duration <= 0:
        return 0.0, 0.0
    interval_duration = np.diff(times)
    violating = float(interval_duration[mask[:-1]].sum())
    return violating, min(1.0, max(0.0, violating / duration))


def road_boundary_collision(frame: pd.DataFrame, csv_path: Path, datasets_dir: Path) -> tuple[bool, str]:
    """Check the ego footprint against the bitmap mask cropped around its path."""
    terminal_collision = int(frame["end"].iloc[-1]) == 3
    ego_margin = max(
        float(frame["locationPoint2Head_ego"].max()),
        float(frame["locationPoint2Rear_ego"].max()),
        float(frame["width_ego"].max()),
    ) + 2.0
    bounds = trajectory_bounds(frame, ["ego"], ego_margin)
    map_patch = load_map_patch(datasets_dir, csv_path, bounds)
    if map_patch is None:
        return terminal_collision, "terminal_code_only"

    image, extent = map_patch
    drivable = image >= 128
    x_scale = image.shape[1] / (extent[1] - extent[0])
    y_scale = image.shape[0] / (extent[3] - extent[2])
    for _, row in frame.iterrows():
        corners = vehicle_corners(row, "ego")
        pixel_x = (corners[:, 0] - extent[0]) * x_scale
        pixel_y = (corners[:, 1] - extent[2]) * y_scale
        if np.any(pixel_x < 0) or np.any(pixel_y < 0) or np.any(pixel_x >= image.shape[1]) or np.any(pixel_y >= image.shape[0]):
            return True, "bitmap_footprint"
        left = max(0, int(math.floor(pixel_x.min())))
        right = min(image.shape[1], int(math.ceil(pixel_x.max())) + 1)
        bottom = max(0, int(math.floor(pixel_y.min())))
        top = min(image.shape[0], int(math.ceil(pixel_y.max())) + 1)
        footprint = Image.new("1", (right - left, top - bottom), 0)
        local_points = [(float(x - left), float(y - bottom)) for x, y in zip(pixel_x, pixel_y)]
        ImageDraw.Draw(footprint).polygon(local_points, fill=1)
        footprint_mask = np.asarray(footprint, dtype=bool)
        if np.any(footprint_mask & ~drivable[bottom:top, left:right]):
            return True, "bitmap_footprint"
    return terminal_collision, "bitmap_footprint"


def zero_score(scene_name: str, expected_time: float, errors: list[str]) -> dict:
    return {
        "scene": scene_name,
        "valid": False,
        "errors": errors,
        "expected_time_s": expected_time,
        "score": {"safety": 0.0, "efficiency": 0.0, "total": 0.0},
    }


def score_csv(
    csv_path: Path,
    expected_time: float,
    datasets_dir: Path,
    ttc_threshold: float = DEFAULT_TTC_THRESHOLD_S,
    lateral_distance_threshold: float = DEFAULT_LATERAL_DISTANCE_THRESHOLD_M,
    lateral_acc_threshold: float = DEFAULT_LATERAL_ACCEL_THRESHOLD_MPSS,
) -> dict:
    csv_path = csv_path.expanduser().resolve()
    scene_name = csv_path.stem.removesuffix("_result")
    if expected_time <= 0:
        return zero_score(scene_name, expected_time, ["期望耗时必须大于 0"])
    frame = load_result(csv_path)
    errors = validate_result(frame)
    if errors:
        return zero_score(scene_name, expected_time, errors)

    times = frame.index.to_numpy(float)
    actual_time = float(times[-1] - times[0])
    traffic = traffic_metrics(frame, ttc_threshold, lateral_distance_threshold)
    lateral_acc = lateral_acceleration(frame)
    lateral_acc_violation = lateral_acc > lateral_acc_threshold
    road_collision, road_check = road_boundary_collision(frame, csv_path, datasets_dir)
    terminal = int(frame["end"].iloc[-1])
    vehicle_collision = traffic["vehicle_collision"] or terminal == 2

    ttc_duration, ttc_ratio = violation_duration(traffic["ttc_violation"], times)
    lateral_duration, lateral_ratio = violation_duration(traffic["lateral_violation"], times)
    lateral_acc_duration, lateral_acc_ratio = violation_duration(lateral_acc_violation, times)
    components = {
        "ttc": 20.0 * (1.0 - ttc_ratio),
        "lateral_distance": 20.0 * (1.0 - lateral_ratio),
        "lateral_acceleration": 20.0 * (1.0 - lateral_acc_ratio),
    }
    safety_score = 0.0 if vehicle_collision or road_collision else sum(components.values())
    completion_score = 20.0 if terminal == 4 else 0.0
    time_score = 20.0 * min(1.0, expected_time / actual_time) if actual_time > 0 else 0.0
    efficiency_score = completion_score + time_score
    total_score = safety_score + efficiency_score

    def rounded(value):
        return None if value is None else round(float(value), 4)

    return {
        "scene": scene_name,
        "valid": True,
        "errors": [],
        "terminal_status": terminal,
        "expected_time_s": rounded(expected_time),
        "actual_time_s": rounded(actual_time),
        "parameters": {
            "ttc_threshold_s": ttc_threshold,
            "lateral_distance_threshold_m": lateral_distance_threshold,
            "lateral_acc_threshold_mps2": lateral_acc_threshold,
        },
        "metrics": {
            "vehicle_collision": bool(vehicle_collision),
            "road_boundary_collision": bool(road_collision),
            "road_boundary_check": road_check,
            "minimum_ttc_s": rounded(traffic["minimum_ttc_s"]),
            "ttc_violation_duration_s": rounded(ttc_duration),
            "ttc_violation_ratio": rounded(ttc_ratio),
            "minimum_lateral_distance_m": rounded(traffic["minimum_lateral_distance_m"]),
            "lateral_distance_violation_duration_s": rounded(lateral_duration),
            "lateral_distance_violation_ratio": rounded(lateral_ratio),
            "maximum_lateral_acc_mps2": rounded(lateral_acc.max()),
            "lateral_acc_violation_duration_s": rounded(lateral_acc_duration),
            "lateral_acc_violation_ratio": rounded(lateral_acc_ratio),
            "task_completed": terminal == 4,
        },
        "components": {
            "ttc_safety": rounded(components["ttc"]),
            "lateral_distance_safety": rounded(components["lateral_distance"]),
            "lateral_acceleration_safety": rounded(components["lateral_acceleration"]),
            "task_completion": rounded(completion_score),
            "task_time": rounded(time_score),
        },
        "score": {
            "safety": rounded(safety_score),
            "efficiency": rounded(efficiency_score),
            "total": rounded(total_score),
        },
    }


def resolve_results(inputs: Sequence[Path]) -> list[Path]:
    results = []
    for value in inputs:
        path = value.expanduser().resolve()
        matches = sorted(path.glob("*_result.csv")) if path.is_dir() else [path]
        for match in matches:
            if not match.is_file():
                raise FileNotFoundError(f"结果 CSV 不存在：{match}")
            if match not in results:
                results.append(match)
    if not results:
        raise FileNotFoundError("没有找到 *_result.csv")
    return results


def load_expected_times(path: Path) -> dict[str, float]:
    with path.expanduser().resolve().open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("期望耗时文件必须是 JSON 对象")
    return {_normalize_scene_name(str(key)): float(value) for key, value in data.items()}


def _normalize_scene_name(value: str) -> str:
    return value.removesuffix(".csv").removesuffix("_result")


def expected_time_for(csv_path: Path, args, mapping) -> float:
    if args.expected_time is not None:
        return float(args.expected_time)
    scene_name = csv_path.stem.removesuffix("_result")
    candidates = (scene_name, csv_path.name, csv_path.stem)
    for candidate in candidates:
        normalized = _normalize_scene_name(candidate)
        if normalized in mapping:
            return float(mapping[normalized])
    raise KeyError(f"期望耗时文件中缺少场景：{scene_name}")


def score(args: argparse.Namespace) -> tuple[dict, Path]:
    for value, name in (
        (args.ttc_threshold, "TTC 阈值"),
        (args.lateral_distance_threshold, "侧向距离阈值"),
        (args.lateral_acc_threshold, "侧向加速度阈值"),
    ):
        if value <= 0:
            raise ValueError(f"{name}必须大于 0")
    results = resolve_results(args.results)
    mapping = {}
    if args.expected_time is None:
        mapping = load_expected_times(args.expected_times or DEFAULT_EXPECTED_TIMES)
    scene_reports = [
        score_csv(
            csv_path,
            expected_time_for(csv_path, args, mapping),
            args.datasets.expanduser().resolve(),
            args.ttc_threshold,
            args.lateral_distance_threshold,
            args.lateral_acc_threshold,
        )
        for csv_path in results
    ]
    totals = [report["score"]["total"] for report in scene_reports]
    report = {
        "rule": "mine-sim safety-60 efficiency-40",
        "aggregation": "arithmetic mean across scenes",
        "scene_count": len(scene_reports),
        "valid_scene_count": sum(report["valid"] for report in scene_reports),
        "aggregate_score": round(float(np.mean(totals)), 4),
        "scenes": scene_reports,
    }
    default_output = (
        results[0].with_name(f"{results[0].stem}_score.json") if len(results) == 1 else Path.cwd() / "score_report.json"
    )
    output_path = (args.output or default_output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    return report, output_path


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        report, output_path = score(args)
    except (FileNotFoundError, KeyError, ValueError) as exc:
        parser.error(str(exc))
    for scene in report["scenes"]:
        print(
            f"{scene['scene']}: safety={scene['score']['safety']:.2f}, "
            f"efficiency={scene['score']['efficiency']:.2f}, total={scene['score']['total']:.2f}"
        )
    print(f"aggregate: {report['aggregate_score']:.2f}")
    print(f"###log### 评分报告：{output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
