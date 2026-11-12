"""mine-sim simulator command-line entry point."""

from __future__ import annotations

import argparse
import csv
import fnmatch
import os
import sys
import time
import traceback
from pathlib import Path
from typing import Iterable, Sequence


DEVKIT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = DEVKIT_ROOT.parent


def _add_import_paths() -> None:
    for path in (
        DEVKIT_ROOT,
        DEVKIT_ROOT / "common",
        DEVKIT_ROOT / "dynamic_scenes",
        DEVKIT_ROOT / "map_expansion",
    ):
        value = str(path)
        if value not in sys.path:
            sys.path.insert(0, value)


def _configure_matplotlib(mode: str, outputs_dir: Path) -> None:
    """Select a backend before any module imports matplotlib.pyplot."""
    cache_dir = outputs_dir / ".matplotlib"
    cache_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache_dir))

    import matplotlib

    if mode in {"none", "save"}:
        matplotlib.use("Agg")
    elif matplotlib.get_backend().lower() in {"agg", "pdf", "svg", "ps", "cairo", "template"}:
        raise RuntimeError(
            "实时可视化需要桌面图形环境，当前 Matplotlib 后端为 "
            f"{matplotlib.get_backend()!r}。请改用 --visualize save 或 --visualize none。"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="运行 mine-sim 回放仿真并输出场景 CSV。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--inputs", type=Path, default=PROJECT_ROOT / "inputs", help="用户待测场景 JSON 所在目录")
    parser.add_argument("--datasets", type=Path, default=PROJECT_ROOT / "datasets", help="地图数据目录")
    parser.add_argument("--outputs", type=Path, default=PROJECT_ROOT / "outputs", help="CSV 与运行摘要输出目录")
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        metavar="NAME_OR_GLOB",
        help="指定场景文件、文件名或通配符；可重复。未指定时测试 inputs 中的全部 JSON",
    )
    parser.add_argument("--list-scenarios", action="store_true", help="列出 inputs 中公开场景后退出")
    parser.add_argument("--visualize", choices=("none", "show", "save"), default="none", help="关闭、实时显示或保存逐帧 PNG")
    parser.add_argument("--skip-existing", action="store_true", help="跳过 outputs 中已经存在结果 CSV 的场景")
    parser.add_argument("--max-scenes", type=int, help="最多运行多少个场景，适合快速验证")
    parser.add_argument("--continue-on-error", action="store_true", help="单个场景失败后继续测试其余场景")
    return parser


def _json_files(directory: Path) -> list[Path]:
    if not directory.is_dir():
        return []
    return sorted(
        path.resolve()
        for path in directory.iterdir()
        if path.is_file() and path.name.startswith("Scenario-") and path.suffix.lower() == ".json"
    )


def resolve_scenario_files(inputs_dir: Path, selectors: Sequence[str]) -> list[Path]:
    """Resolve explicit selectors or return every JSON directly under inputs."""
    if not selectors:
        return _json_files(inputs_dir)

    available = _json_files(inputs_dir)

    selected: list[Path] = []
    for selector in selectors:
        direct = Path(selector).expanduser()
        if direct.is_file():
            matches = [direct.resolve()]
        else:
            matches = [
                path
                for path in available
                if fnmatch.fnmatch(path.name, selector)
                or fnmatch.fnmatch(path.stem, selector)
                or fnmatch.fnmatch(path.stem.removeprefix("Scenario-"), selector)
            ]
        if not matches:
            raise FileNotFoundError(f"未找到场景选择器对应的 JSON：{selector}")
        for path in matches:
            if path not in selected:
                selected.append(path)
    return selected


def _write_summary(rows: Iterable[dict], outputs_dir: Path) -> Path:
    summary_path = outputs_dir / "run_summary.csv"
    rows = list(rows)
    if not rows and summary_path.exists():
        return summary_path
    fieldnames = ("scenario", "end", "steps", "simulated_time_s", "wall_time_s", "result_csv", "error")
    with summary_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return summary_path


def run(args: argparse.Namespace) -> int:
    datasets_dir = args.datasets.expanduser().resolve()
    inputs_dir = args.inputs.expanduser().resolve()
    outputs_dir = args.outputs.expanduser().resolve()

    if not datasets_dir.is_dir():
        raise FileNotFoundError(f"数据集目录不存在：{datasets_dir}")

    if args.list_scenarios:
        for path in _json_files(inputs_dir):
            print(path.name)
        return 0

    inputs_dir.mkdir(parents=True, exist_ok=True)
    outputs_dir.mkdir(parents=True, exist_ok=True)
    scenario_files = resolve_scenario_files(inputs_dir, args.scenario)
    if not scenario_files:
        raise FileNotFoundError(
            f"{inputs_dir} 中没有 Scenario-*.json。请复制公开场景到该目录，或使用 "
            "--scenario Scenario-dapai_intersection_1_3_10.json 选择指定场景。"
        )
    if args.max_scenes is not None:
        if args.max_scenes <= 0:
            raise ValueError("--max-scenes 必须大于 0")
        scenario_files = scenario_files[: args.max_scenes]

    _configure_matplotlib(args.visualize, outputs_dir)
    _add_import_paths()

    import matplotlib.pyplot as plt

    from dynamic_scenes.env import Env
    from dynamic_scenes.scenarioOrganizer import ScenarioOrganizer
    from planner import Planner
    from predictor import Predictor
    from simple_control import MotionController

    organizer = ScenarioOrganizer()
    organizer.load(
        str(inputs_dir),
        str(outputs_dir),
        str(datasets_dir),
        scenario_files=scenario_files,
        skip_existing=args.skip_existing,
    )
    visualize = args.visualize != "none"
    save_frames = args.visualize == "save"
    for scenario in organizer.scenario_list:
        scenario["test_settings"]["visualize"] = visualize
        scenario["test_settings"]["save_fig_whitout_show"] = save_frames

    print(f"###log### 待测试场景数：{len(organizer.scenario_list)}")
    print(f"###log### 结果目录：{outputs_dir}")
    summaries: list[dict] = []
    failures = 0

    while True:
        scenario = organizer.next()
        if scenario is None:
            break
        scene_name = scenario["data"]["scene_name"]
        print(f"\n###log### 开始测试：{scene_name}")
        started = time.perf_counter()
        steps = 0
        observation = None
        error_text = ""
        try:
            environment = Env()
            observation, traj = environment.make(scenario=scenario)
            predictor = Predictor(time_horizon=5.0)
            planner = Planner(observation)
            path_planned, _ = planner.process(observation)
            controller = MotionController()
            while observation["test_setting"]["end"] == -1:
                predicted_trajectories, _ = predictor.predict(
                    observation,
                    traj,
                    predictor_type="CACV_PREDICTOR",
                )
                action = controller.process(
                    observation["vehicle_info"],
                    path_planned,
                    predicted_trajectories,
                    observation,
                )
                observation = environment.step(action, predicted_trajectories, observation, traj)
                steps += 1
        except Exception as exc:  # Keep a partial CSV and report the failing scene.
            failures += 1
            error_text = f"{type(exc).__name__}: {exc}"
            print(f"###error### {scene_name}: {error_text}", file=sys.stderr)
            traceback.print_exc()
            if not args.continue_on_error:
                summaries.append(
                    _summary_row(scene_name, observation, steps, started, outputs_dir, error_text)
                )
                plt.close("all")
                break
        summaries.append(_summary_row(scene_name, observation, steps, started, outputs_dir, error_text))
        plt.close("all")

    summary_path = _write_summary(summaries, outputs_dir)
    print(f"\n###log### 测试完成：成功 {len(summaries) - failures}，失败 {failures}")
    print(f"###log### 运行摘要：{summary_path}")
    return 1 if failures else 0


def _summary_row(scene_name, observation, steps, started, outputs_dir, error_text):
    settings = observation["test_setting"] if observation else {}
    return {
        "scenario": scene_name,
        "end": settings.get("end", -1),
        "steps": steps,
        "simulated_time_s": round(float(settings.get("t", 0)), 4),
        "wall_time_s": round(time.perf_counter() - started, 4),
        "result_csv": str(outputs_dir / f"{scene_name}_result.csv"),
        "error": error_text,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
