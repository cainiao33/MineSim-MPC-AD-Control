# MineSim-MPC-AD-Control

矿区自动驾驶 **决策规划控制** 赛道策略开源包：以 **v63** 生产叠加层为主，附方法叙事与演示视频。

<p align="center">
  <a href="README.md"><img src="https://img.shields.io/badge/语言-简体中文-red?style=for-the-badge" alt="简体中文"></a>
  <a href="README_en.md"><img src="https://img.shields.io/badge/语言-English-blue?style=for-the-badge" alt="English"></a>
</p>

> **形态**：策略叠加仓（覆盖官方 mine_sim 的 `devkit/`），**不**重分发官方地图 / 场景输入 / Docker 镜像。  
> **线上可绑定基线**：**v58 ≈ 96.12**（183 场均分 96.121366…）  
> **本地完整最佳**：**v63** 公开 40 ≈ **97.57** / 已知 43 ≈ **97.10**（零车撞、零压界；**非**线上认证包）  
> **许可证**：AGPL-3.0（与上游 mine_sim 同族）— 见 [LICENSE](LICENSE) / [NOTICE](NOTICE)

<p align="center">
  <img src="assets/v63_jiangtong_6_3_3_480.gif" alt="v63 跟随镜头演示（jiangtong 6_3_3）" width="480"/>
</p>

完整跟拍 MP4（11 场 ×1x/0.5x + 混剪，约 144MB）：  
**[Release `v63-demo-videos`](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/tag/v63-demo-videos)**

---

## 成绩口径（不要混写）

| 口径 | 版本 | 分数 |
|---|---|---:|
| 线上 183 场 | v58-full-ff | **96.121366…**（显示 ≈96.12） |
| 本地公开 40 | v63 | **97.572355** |
| 本地已知 43 | v63 | **97.096258…** |

本地 43 ≠ 线上 183。提分曲线与角色归因：

<p align="center">
  <img src="assets/score_chain_guikong.png" alt="规控提分链" width="720"/>
</p>
<p align="center">
  <img src="assets/score_chain_roles.png" alt="破纪录/探针角色" width="720"/>
</p>

细节见 [SCORES.md](SCORES.md)。

---

## 方法一页纸

1. **回放时序论**：背景车非反应回放；ego 乱变速/换径会自投冲突窗；恒速贴中线是早期基座。  
2. **三层自适应银行**：路径 modes → 速度/IDM/让行 → alat/rot/look 安全层；单变量 A/B，赢家入库、输家回退。  
3. **生产栈**：SAS 同码闭环候选 → STPP/重定时前馈 → v63 全轨迹 SQP + 目标多边形派发（保留原安全门与滞后）。  

展开：[METHOD.md](METHOD.md) · 模块图：[ARCHITECTURE.md](ARCHITECTURE.md)

本仓附带**完整** `scen_config.json`（183 场 modes 与各轴键表），作为「场景自适应策略选择」方法本身的一部分。

---

## 快速开始

```bash
# 1) 自备官方 mine_sim（含 inputs/、datasets/maps/、可运行环境）
# 2) 叠加本仓策略
git clone https://github.com/cainiao33/MineSim-MPC-AD-Control.git
cd MineSim-MPC-AD-Control
bash scripts/overlay_into_minesim.sh /path/to/official_mine_sim

# 3) 在官方树内跑公开场景（解释器用你的 mine_sim 环境）
cd /path/to/official_mine_sim
python devkit/__main__.py \
  --scenario Scenario-dapai_intersection_1_3_10.json \
  --visualize none
```

脚本会覆盖 `devkit/` 中的 NEW/MOD 文件，并在首次覆盖时备份原文件到 `.mpc_ad_overlay_backup/`。

可选环境变量（路径已去本机硬编码）：

| 变量 | 作用 |
|---|---|
| `MINE_SCEN_CONFIG` / `MINE_CFG` | 显式指定 `scen_config.json` |
| `MINE_CENTERLINE_DIR` | 中线表目录 |
| `MINE_EXPECTED_JSON` | 可选期望耗时表 |

默认会自动读 `devkit/centerlines/scen_config.json`。

---

## 仓库内容

```
overlay/devkit/     # 58 个策略文件（54 NEW + 4 MOD），可直接覆盖
assets/             # 提分图 + 预览 gif
scripts/            # overlay_into_minesim.sh
MANIFEST.json       # 逐文件 sha256（3 个路径清洗文件已标注）
METHOD.md / ARCHITECTURE.md / SCORES.md
```

**不包含**：官方 `inputs/`、地图、隐藏场轨迹、Docker tar、感知赛道资产。

文件清单与哈希见 [MANIFEST.json](MANIFEST.json)。

---

## 引用与上游

- 仿真器上游：MineSim / mine_sim（AGPL-3.0）  
- 本叠加层：竞赛规控生产线 v63；线上叙事基线 v58  

若你基于本仓改动并对外提供网络服务，请遵守 AGPL-3.0 的源码提供义务。

---

## 演示视频列表（Release）

| 内容 | 说明 |
|---|---|
| 11 场跟随镜头 1x / 0.5x | 1080²，blend 插帧 30fps |
| `v63_highlights_1080.mp4` | 86s 混剪 |
| 仓内 gif | `assets/v63_jiangtong_6_3_3_480.gif` |

视频对应 v63 封箱行为的渲染展示（镜头裁剪不影响仿真）。
