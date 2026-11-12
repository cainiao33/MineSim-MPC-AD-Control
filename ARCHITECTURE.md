# 架构说明

## 运行回路（官方 mine_sim）

```
observation
  → Predictor.predict
  → MotionController / simple_control.process
  → action（加速度、转向）
```

本仓根目录即为可运行的 mine_sim V1.1（含 `inputs/`、`datasets/`）；策略已打进 `devkit/`。环境与评分器保持官方逻辑。

## 策略相关目录

```
devkit/                   # 已打补丁，直接运行
  planner.py              # 路径模式 + 中线 / 局部修正表选择
  predictor.py            # 预测 + SAS 钩子
  simple_control.py       # 跟踪、IDM / 侧向加速度 / 转角 / 预瞄 / 让行，以及 SAS、STPP 入口
  centerlines/
    scen_config.json      # 183 场场景参数表（完整）
    centerlines_*.json    # 官方中线表 + v2s 局部修正表
  sas_core.py / sas_hook.py
  stpp_*.py / executed_retiming*.py
  control_full_trace_*.py / control_live_goal_polygon.py / ...
overlay/devkit/           # 同上策略增量镜像（可选打到另一份官方树）
```

## 模块分组

| 分组 | 代表文件 | 作用 |
|---|---|---|
| 路径 | `planner.py`、`centerlines/*`、`bitmap_route.py`、`bootstrap_route.py` | 场景模式与走廊几何 |
| 跟踪控制 | `simple_control.py`、`predictor.py` | 速度 / 安全场景参数与控制 |
| SAS | `sas_hook.py`、`sas_core.py`、`score_embed.py` | 闭环候选搜索 |
| STPP / 重定时 | `stpp_hook.py`、`stpp_geometry.py`、`executed_retiming*.py` | 执行几何与时序调整 |
| 全轨迹 SQP | `control_full_trace_*.py`、`full_trace_*.py`、`control_live_goal_polygon.py` | v63 目标区域精修 |
| 试验与计量 | `*_trials.py`、`*_work_meter.py` | 搜索预算与准入 |

## 配置查找顺序（可移植）

开源版已去掉本机绝对路径，大致顺序为：

1. `MINE_CFG` / `MINE_SCEN_CONFIG`（显式文件）
2. `devkit/centerlines/scen_config.json`（与代码同目录）
3. `./outputs/scen_config.json`（当前工作目录回退）

中线表：`MINE_CENTERLINE_DIR` → `devkit/centerlines/` → `./outputs/`。

可选期望耗时：`MINE_EXPECTED_JSON` → `<mine_sim>/inputs/expected_times.json`。

## 安装方式

主路径：克隆本仓 → `pip install -r setup/requirements.txt` → `python devkit/__main__.py ...`。

可选：已有单独官方树时，`bash scripts/overlay_into_minesim.sh /path/to/official_mine_sim` 仅覆盖策略文件（首次备份到 `.mpc_ad_overlay_backup/`）。
