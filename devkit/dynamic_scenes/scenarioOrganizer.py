"""Scenario discovery and iteration."""

from __future__ import annotations

import os
import sys
from copy import deepcopy
from pathlib import Path
from typing import Iterable


DEVKIT_ROOT = Path(__file__).resolve().parent.parent
if str(DEVKIT_ROOT) not in sys.path:
    sys.path.insert(0, str(DEVKIT_ROOT))

from mine_sim_test_conf import config


class ScenarioOrganizer:
    """Read local replay scenarios and return them in deterministic order."""

    def __init__(self, use_socket: bool = False):
        self.socket_mode = use_socket
        self.test_mode = ""
        self.scenario_list: list[dict] = []
        self.test_num = 0
        self.config = {}

    def load(
        self,
        dir_inputs: str,
        dir_outputs: str,
        dir_datasets: str,
        scenario_files: Iterable[os.PathLike | str] | None = None,
        skip_existing: bool | None = None,
    ) -> None:
        """Load replay scenarios from explicit files or the inputs directory."""
        inputs_dir = Path(dir_inputs).expanduser().resolve()
        outputs_dir = Path(dir_outputs).expanduser().resolve()
        datasets_dir = Path(dir_datasets).expanduser().resolve()
        outputs_dir.mkdir(parents=True, exist_ok=True)

        self.config = deepcopy(config)
        self.test_mode = self.config["test_settings"]["mode"]
        self.config["test_settings"].setdefault("visualize", False)
        self.config["test_settings"].setdefault("save_fig_whitout_show", False)
        if skip_existing is not None:
            self.config["test_settings"]["skip_exist_scene"] = skip_existing
        self.config["test_settings"].setdefault("skip_exist_scene", False)

        if self.test_mode != "replay":
            raise ValueError(f"当前版本只支持 replay 模式，收到：{self.test_mode}")

        if scenario_files is None:
            if not inputs_dir.is_dir():
                raise FileNotFoundError(f"输入目录不存在：{inputs_dir}")
            paths = sorted(
                path
                for path in inputs_dir.iterdir()
                if path.is_file() and path.name.startswith("Scenario-") and path.suffix.lower() == ".json"
            )
        else:
            paths = sorted(Path(path).expanduser().resolve() for path in scenario_files)

        self.scenario_list = []
        for path in paths:
            if not path.is_file() or path.suffix.lower() != ".json":
                raise FileNotFoundError(f"场景 JSON 不存在：{path}")
            scene_name = path.stem.removeprefix("Scenario-")
            result_path = outputs_dir / f"{scene_name}_result.csv"
            if self.config["test_settings"]["skip_exist_scene"] and result_path.exists():
                print(f"###log### 跳过已有结果：{scene_name}")
                continue

            location_prefix = scene_name.split("_", 1)[0]
            locations = {
                "jiangtong": "jiangxi_jiangtong",
                "dapai": "guangdong_dapai",
            }
            if location_prefix not in locations:
                raise ValueError(f"无法从场景名判断地图位置：{scene_name}")

            scenario = deepcopy(self.config)
            scenario["file_info"] = {
                "dir_inputs": str(inputs_dir),
                "dir_outputs": str(outputs_dir),
                "dir_visualizations": str(outputs_dir / "visualizations"),
                "dir_datasets": str(datasets_dir),
                "location": locations[location_prefix],
            }
            scenario["data"] = {
                "scene_name": scene_name,
                "dir_scene_file": str(path),
            }
            self.scenario_list.append(scenario)

        self.test_num = len(self.scenario_list)

    def next(self):
        """Return the next scenario or ``None`` when all tests are complete."""
        return self.scenario_list.pop(0) if self.scenario_list else None

    def add_result(self, concrete_scenario: dict, res: float) -> None:
        """Compatibility hook reserved for future interactive/adaptive modes."""
        return None
