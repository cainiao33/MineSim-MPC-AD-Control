# MineSim-MPC-AD — 矿区自动驾驶规控策略（基于 MineSim）

<p align="center">
  <a href="README.md"><img src="https://img.shields.io/badge/切换语言-简体中文-red?style=for-the-badge" alt="简体中文"></a>
  <a href="README_en.md"><img src="https://img.shields.io/badge/切换语言-English-blue?style=for-the-badge" alt="English"></a>
</p>

> **赛题**：[琶洲算法大赛 × 智能驾驶生态赛](https://deepvision.aicompetition-pz.com/#/homeDetail?id=2095413633562579532) — **决策规划控制**任务（MineSim 回放仿真）。大赛官网：[aicompetition-pz.com](https://www.aicompetition-pz.com/) · 赛题详情页：[deepvision 平台](https://deepvision.aicompetition-pz.com/#/homeDetail?id=2095413633562579532)。
>
> **概述**：面向 MineSim / mine_sim 回放仿真的**策略叠加层**——场景自适应配置银行 + SAS 同码闭环候选 + STPP/重定时前馈 + v63 全轨迹 SQP / 目标多边形派发。覆盖官方 `devkit/`，不改 Env / 地图 / 评分器。
>
> **成绩**：线上可绑定包 **v58 ≈ 96.12**（183 场均分 96.121366…）· 本地完整最佳 **v63** 公开 40 ≈ **97.57** / 已知 43 ≈ **97.10**（零车撞、零压界；**非**线上认证包）。
>
> **怎么跑**：须自备官方 mine_sim（含 `inputs/` 与地图）。本仓只发策略文件；一键覆盖后即可跑公开场景：
>
> ```bash
> git clone https://github.com/cainiao33/MineSim-MPC-AD-Control.git
> bash MineSim-MPC-AD-Control/scripts/overlay_into_minesim.sh /path/to/official_mine_sim
> cd /path/to/official_mine_sim
> python devkit/__main__.py --scenario Scenario-dapai_intersection_1_3_10.json --visualize none
> ```
>
> 演示视频：[跟随镜头](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/tag/v63-demo-videos) · [全景视角](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/tag/v63-demo-videos-wide)；任务与数据：[TASK_AND_DATA.md](TASK_AND_DATA.md)；方法：[METHOD.md](METHOD.md) / [ARCHITECTURE.md](ARCHITECTURE.md)。

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: AGPL-3.0](https://img.shields.io/badge/License-AGPL--3.0-blue.svg)](LICENSE)
[![琶洲算法大赛](https://img.shields.io/badge/琶洲算法大赛-智能驾驶生态赛-orange.svg)](https://deepvision.aicompetition-pz.com/#/homeDetail?id=2095413633562579532)
[![MineSim Overlay](https://img.shields.io/badge/MineSim-strategy_overlay-green.svg)](NOTICE)
[![Online v58](https://img.shields.io/badge/online_v58-96.12-brightgreen.svg)](SCORES.md)

---

## 📖 项目简介

本项目是 **[琶洲算法大赛 × 智能驾驶生态赛](https://deepvision.aicompetition-pz.com/#/homeDetail?id=2095413633562579532)** 中**决策规划控制**赛道方案的开源切片：在命题方 MineSim 回放基准上，用可解释的规则栈与实证银行把公开集与线上分从早期基线抬到 **v58 ≈ 96.12**（线上）/ **v63 ≈ 97.1**（本地已知 43）。

技术路线：**不替换仿真器本体**——Env、地图、官方评分脚本保持原样；选手侧只改 `planner` / `predictor` / `simple_control`，并叠加 SAS、STPP、全轨迹 SQP 等模块。完整 183 场 `scen_config.json` 随仓发布，作为「场景自适应策略选择」方法的一部分。

### 本项目的改动（相对官方 mine_sim `devkit/`）

- **🗺 路径层银行**：逐场 `modes`（base / center / hybrid）+ 离线中线表 + `centerlines_*_v2s.json` 外科表
- **⏱ 速度 / 跟车层**：`vel0` / `vprof` / IDM / yield 等场景键；单变量 A/B，赢家入库、输家回退
- **🛡 安全跟踪层**：`alat` / `rot` / look 等封顶与预瞄；与评分侧加速度、转角约束对齐
- **🔁 SAS v2**：同码闭环候选 rollout（原 plant / 控制器 / 评分器），按 commit 窗选优
- **📐 STPP + 重定时前馈**：在认证安全模型上整形执行时序（v58 线上认证主链）
- **🎯 v63 全轨迹 SQP + 目标多边形**：对原生安全选中表做目标面感知精修，保留原 `HYST`，不强推新政策

> 说明：官方 `inputs/`、HD 地图、隐藏场轨迹、Docker 提交镜像**不随本仓分发**，须按赛题官方渠道自备。上游许可证为 AGPL-3.0，见 [LICENSE](LICENSE) / [NOTICE](NOTICE)。

---

## 🏆 竞赛成果

| 项 | 内容 |
|------|------|
| **赛事** | [琶洲算法大赛 × 智能驾驶生态赛](https://deepvision.aicompetition-pz.com/#/homeDetail?id=2095413633562579532) |
| **赛道** | 决策规划控制（初赛 B / 决赛 B） |
| **官方链接** | [赛题详情](https://deepvision.aicompetition-pz.com/#/homeDetail?id=2095413633562579532) · [大赛官网](https://www.aicompetition-pz.com/) |

| 指标 | 成绩 |
|------|------|
| **线上可绑定基线** | **v58-full-ff ≈ 96.12**（183 场均分 96.121366…） |
| **本地公开 40** | **v63 = 97.572355** |
| **本地已知 43** | **v63 = 97.096258…**（全到达 · 零车撞 · 零压界） |
| **早期公开集阶梯** | baseline ~74 → fin2 ~90.23（本地公开 40，历史过程） |
| **平台首评 → 认证链** | 首评 82.00 → … → v54 ≈95.996 → **v58 ≈96.12** |

**口径铁律**：本地 43 ≠ 线上 183；v63 是本地完整最佳，**未**作为线上认证包绑定。细节与提分链见 [SCORES.md](SCORES.md)。

<p align="center">
  <img src="assets/score_chain_guikong.png" alt="规控线上提分链" width="720"/>
</p>
<p align="center"><sub>提分曲线（破纪录点）· 角色归因图见下</sub></p>

<p align="center">
  <img src="assets/score_chain_roles.png" alt="破纪录 / 探针 / 回收角色归因" width="720"/>
</p>

---

## 📋 任务场景与规控数据集

| 项 | 内容 |
|------|------|
| **赛事** | [琶洲算法大赛 × 智能驾驶生态赛](https://deepvision.aicompetition-pz.com/#/homeDetail?id=2095413633562579532) |
| **任务** | 基于感知结果 + 位姿 / 导航 / 动力学，在无信号灯路口与动态交互场景输出转向、油门、制动 |
| **评分** | 安全性 50% + 效率性 30% + 路径完成度 20% |
| **数据集（官方）** | 位姿序列、导航指令、动力学状态、感知结果；非结构化道路无信号灯路口；初/决赛内容一致、分布不同 |
| **公开包** | 2 图（大排 / 江铜）· **40** 公开场景 JSON · baseline + README；线上评测约 **183** 场 |
| **本仓演示 11 场** | dapai `1_3_10/1_3_15/2_3_5/3_2_2` · jiangtong `1_5_3/1_5_4/5_4_1/6_2_5/6_3_3/9_3_2/9_3_3` |

完整说明（含机制要点与官方链接）：[TASK_AND_DATA.md](TASK_AND_DATA.md)。官方数据须自行申领，**不**随本仓分发。

---

## 🎥 演示效果

两种渲染视角，行为同源（v63 封箱轨迹）；镜头后处理不影响仿真。

| 视角 | 说明 | 完整 MP4 |
|------|------|----------|
| **跟随镜头** | 锁定自车 · EMA 跟拍 · 1080²，运动充满画面 | [Release `v63-demo-videos`](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/tag/v63-demo-videos)（23 文件，约 144MB） |
| **全景 / 地图视角** | 1500² 宽幅，看清路网、背景车回放与整段路线 | [Release `v63-demo-videos-wide`](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/tag/v63-demo-videos-wide)（`wide_*`，约 55MB） |

### 跟随镜头预览

<p align="center">
  <img src="assets/v63_follow_dapai_1_3_10_480.gif" alt="follow dapai 1_3_10" width="24%">
  <img src="assets/v63_follow_dapai_3_2_2_480.gif" alt="follow dapai 3_2_2" width="24%">
  <img src="assets/v63_follow_jiangtong_1_5_4_480.gif" alt="follow jiangtong 1_5_4" width="24%">
  <img src="assets/v63_follow_jiangtong_6_3_3_480.gif" alt="follow jiangtong 6_3_3" width="24%">
</p>
<p align="center"><sub>左→右：dapai 1_3_10 · dapai 3_2_2 · jiangtong 1_5_4 · jiangtong 6_3_3（到达段预览）</sub></p>

混剪：[`v63_highlights_1080.mp4`](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_highlights_1080.mp4)

### 全景 / 地图视角预览

<p align="center">
  <img src="assets/v63_wide_dapai_1_3_10_480.gif" alt="wide dapai 1_3_10" width="24%">
  <img src="assets/v63_wide_jiangtong_1_5_4_480.gif" alt="wide jiangtong 1_5_4" width="24%">
  <img src="assets/v63_wide_jiangtong_6_3_3_480.gif" alt="wide jiangtong 6_3_3" width="24%">
  <img src="assets/v63_wide_highlights_preview_480.gif" alt="wide highlights preview" width="24%">
</p>
<p align="center"><sub>左→右：dapai 1_3_10 · jiangtong 1_5_4 · jiangtong 6_3_3 · 全景混剪节选</sub></p>

全景混剪：[`wide_v63_highlights_1080.mp4`](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_highlights_1080.mp4)

### 成片清单（11 场 × 双倍速 × 双视角）

每场均有 **1x 实时** 与 **0.5x 慢放**。跟随镜头文件名无前缀；全景文件名带 `wide_`。

| 场景 | 地图 | 跟随 1x | 全景 1x |
|------|------|---------|--------|
| 1_3_10 | 大排 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_dapai_intersection_1_3_10_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_dapai_intersection_1_3_10_1x.mp4) |
| 1_3_15 | 大排 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_dapai_intersection_1_3_15_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_dapai_intersection_1_3_15_1x.mp4) |
| 2_3_5 | 大排 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_dapai_intersection_2_3_5_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_dapai_intersection_2_3_5_1x.mp4) |
| 3_2_2 | 大排 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_dapai_intersection_3_2_2_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_dapai_intersection_3_2_2_1x.mp4) |
| 1_5_3 | 江铜 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_jiangtong_intersection_1_5_3_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_jiangtong_intersection_1_5_3_1x.mp4) |
| 1_5_4 | 江铜 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_jiangtong_intersection_1_5_4_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_jiangtong_intersection_1_5_4_1x.mp4) |
| 5_4_1 | 江铜 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_jiangtong_intersection_5_4_1_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_jiangtong_intersection_5_4_1_1x.mp4) |
| 6_2_5 | 江铜 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_jiangtong_intersection_6_2_5_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_jiangtong_intersection_6_2_5_1x.mp4) |
| 6_3_3 | 江铜 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_jiangtong_intersection_6_3_3_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_jiangtong_intersection_6_3_3_1x.mp4) |
| 9_3_2 | 江铜 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_jiangtong_intersection_9_3_2_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_jiangtong_intersection_9_3_2_1x.mp4) |
| 9_3_3 | 江铜 | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_jiangtong_intersection_9_3_3_1x.mp4) | [mp4](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_jiangtong_intersection_9_3_3_1x.mp4) |

绿框=自车，蓝框=背景车；红轨=背景真值未来，绿轨=预测。0.5x 与混剪见对应 Release 页。

---

## ⚠️ 复现说明

本仓库定位是**策略方案开源**（叠加层 + 方法叙事 + 演示），不是「克隆即满分」的完整仿真发行版：

- 必须自备**官方 mine_sim** 与竞赛授权数据（地图 / 公开 `inputs/`）
- 线上 183 场隐藏输入与平台 Docker 评测环境**不在本仓**
- 叠加脚本会改写 `devkit/` 中的 NEW/MOD 文件，并在首次覆盖时备份到 `.mpc_ad_overlay_backup/`
- `planner.py` / `simple_control.py` / `sas_hook.py` 已做**路径可移植清洗**（去掉本机绝对路径）；公开 sha 与冻结 lab 生产树在这 3 个文件上不同，见 [MANIFEST.json](MANIFEST.json)

因此：同赛题持有官方包的读者可按下方步骤叠加复现公开场景；其他读者可将本仓当作回放型规控的工程参考（配置银行 + 闭环自仿真 + 执行精修）。

---

## 📦 快速叠加

```bash
# 1) 官方 mine_sim 根目录须已含 devkit/planner.py 与 inputs/
# 2) 叠加本仓
git clone https://github.com/cainiao33/MineSim-MPC-AD-Control.git
bash MineSim-MPC-AD-Control/scripts/overlay_into_minesim.sh /path/to/official_mine_sim

# 3) 在官方树内跑公开场景（使用你的 mine_sim Python 环境）
cd /path/to/official_mine_sim
python devkit/__main__.py \
  --scenario Scenario-dapai_intersection_1_3_10.json \
  --visualize none
```

可选环境变量：

| 变量 | 作用 |
|------|------|
| `MINE_SCEN_CONFIG` / `MINE_CFG` | 显式指定 `scen_config.json` |
| `MINE_CENTERLINE_DIR` | 中线表目录 |
| `MINE_EXPECTED_JSON` | 可选期望耗时表 |

缺省自动读取 `devkit/centerlines/scen_config.json`。

---

## 🧠 方法要点

1. **回放时序论**：背景车非反应回放；ego 乱变速 / 换径会自投冲突窗；恒速贴中线是早期基座。  
2. **三层自适应银行**：路径 → 速度/IDM/让行 → alat/rot/look；单变量 A/B，族级裁决，输家免费回退。  
3. **生产栈**：SAS 同码闭环 → STPP/重定时前馈（v58）→ v63 全轨迹 SQP + 目标多边形派发。

展开阅读：

| 文档 | 内容 |
|------|------|
| [TASK_AND_DATA.md](TASK_AND_DATA.md) | 赛题任务、评分、规控数据集与公开场景 |
| [METHOD.md](METHOD.md) | 四幕方法叙事 |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 模块分组与配置发现顺序 |
| [SCORES.md](SCORES.md) | 线上 / 本地分数分账与里程碑 |
| [MANIFEST.json](MANIFEST.json) | 58 个叠加文件 sha256 |

---

## 📁 仓库结构

```
MineSim-MPC-AD-Control/
├── overlay/devkit/          # 58 策略文件（54 NEW + 4 MOD），可直接覆盖
│   ├── planner.py / predictor.py / simple_control.py
│   ├── centerlines/scen_config.json          # 完整 183 场配置银行
│   ├── centerlines/centerlines_*_v2s.json
│   ├── sas_*.py / stpp_*.py / executed_retiming*.py
│   └── control_full_trace_*.py / ...
├── scripts/overlay_into_minesim.sh
├── assets/                  # 提分图 + 跟随/全景预览 gif
├── TASK_AND_DATA.md         # 任务场景与规控数据集
├── METHOD.md / ARCHITECTURE.md / SCORES.md
├── MANIFEST.json / NOTICE / LICENSE
└── README.md / README_en.md
```

---

## 📄 许可证与上游

- 仿真器上游：MineSim / mine_sim（**AGPL-3.0**）
- 本叠加层随 AGPL-3.0 发布；与官方树合并运行时适用 copyleft 义务
- 归属与「不重分发官方数据」声明见 [NOTICE](NOTICE)

若基于本仓改动并对外提供网络服务，请遵守 AGPL-3.0 的源码提供义务。
