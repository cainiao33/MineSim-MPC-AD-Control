# MineSim-MPC-AD — Mining AD Planning/Control Strategy (on MineSim)

<p align="center">
  <a href="README.md"><img src="https://img.shields.io/badge/Switch_Language-简体中文-red?style=for-the-badge" alt="简体中文"></a>
  <a href="README_en.md"><img src="https://img.shields.io/badge/Switch_Language-English-blue?style=for-the-badge" alt="English"></a>
</p>

> **Competition:** [Pazhou Algorithm Competition × Intelligent Driving Ecosystem Track](https://www.aicompetition-pz.com/topic_detail/56) — **planning / decision / control** (MineSim replay). Site: [aicompetition-pz.com](https://www.aicompetition-pz.com/) · Problem page: [topic_detail/56](https://www.aicompetition-pz.com/topic_detail/56).
>
> **Overview:** Planning/control strategy on official MineSim / mine_sim V1.1 — scene-adaptive parameters, SAS closed-loop search, STPP / retiming, and v63 full-trace SQP with goal-region refinement. This repo **already includes** a runnable tree (`devkit/` + public `inputs/` + maps); **clone once and run**.
>
> **Results:** Online **v58 ≈ 96.12** (183-scene mean 96.121366…) · Local best **v63** public-40 ≈ **97.57** / local-43 ≈ **97.10** (zero collision / curb hit; v63 was **not** the uploaded online package).
>
> **How to run** (no second download of official mine_sim):
>
> ```bash
> git clone https://github.com/cainiao33/MineSim-MPC-AD-Control.git
> cd MineSim-MPC-AD-Control
> pip install -r setup/requirements.txt
> python devkit/__main__.py --scenario Scenario-dapai_intersection_1_3_10.json --visualize none
> ```
>
> Demos: [follow-cam](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/tag/v63-demo-videos) · [wide/map view](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/tag/v63-demo-videos-wide). Task/data: [TASK_AND_DATA_en.md](TASK_AND_DATA_en.md). Method: [METHOD.md](METHOD.md) / [ARCHITECTURE.md](ARCHITECTURE.md).

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: AGPL-3.0](https://img.shields.io/badge/License-AGPL--3.0-blue.svg)](LICENSE)
[![Pazhou Algorithm Competition](https://img.shields.io/badge/Pazhou-Intelligent_Driving-orange.svg)](https://www.aicompetition-pz.com/topic_detail/56)
[![MineSim included](https://img.shields.io/badge/MineSim-V1.1_included-green.svg)](NOTICE)
[![Online v58](https://img.shields.io/badge/online_v58-96.12-brightgreen.svg)](SCORES.md)

---

## 📖 Introduction

Open-source **planning & control** code for the [Pazhou Algorithm Competition × Intelligent Driving Ecosystem Track](https://www.aicompetition-pz.com/topic_detail/56). On MineSim replay, an interpretable rule stack with per-scene parameters raised scores to online **v58 ≈ 96.12** and local **v63 ≈ 97.1** (43 scenes).

Approach: based on official mine_sim V1.1 — **env and scorer stay stock**. Changes live in `planner` / `predictor` / `simple_control`, plus SAS, STPP, and full-trace SQP. Public 40 scenes, both mine maps, and the full 183-scene `scen_config.json` ship in-tree.

### Changes vs stock `devkit/`

- **🗺 Path planning:** per-scene `modes` (base / center / hybrid), offline centerlines, and a local correction table `centerlines_*_v2s.json`
- **⏱ Speed / car-following:** `vel0` / `vprof` / IDM / yield; single-variable A/B — keep what helps, roll back what hurts
- **🛡 Safety tracking:** lateral-acceleration / steering / look-ahead limits aligned with scoring
- **🔁 SAS v2:** closed-loop candidates on the same plant, controller, and scorer; pick the best decision window
- **📐 STPP + retiming:** reshape executed timing under safety constraints (v58 online mainline)
- **🎯 v63 full-trace SQP + goal region:** refine already-safe trajectories near the goal; keep switching hysteresis

> Online **hidden** scenes and the platform judge are out of tree. License: AGPL-3.0 — [LICENSE](LICENSE) / [NOTICE](NOTICE). Upstream guide: [UPSTREAM_README.md](UPSTREAM_README.md).

---

## 🏆 Results

| Item | Content |
|------|------|
| **Competition** | [Pazhou × Intelligent Driving](https://www.aicompetition-pz.com/topic_detail/56) |
| **Track** | Planning / decision / control (Preliminary B / Final B) |
| **Links** | [Problem detail](https://www.aicompetition-pz.com/topic_detail/56) · [Site](https://www.aicompetition-pz.com/) |

| Metric | Value |
|------|------|
| **Online package** | **v58 ≈ 96.12** (183-scene mean 96.121366…) |
| **Local public 40** | **v63 = 97.572355** |
| **Local 43** | **v63 = 97.096258…** (all arrive · zero crash · zero curb) |
| **Early public set** | baseline ~74 → ~90.23 (local public 40, process) |
| **Online progression** | first score 82.00 → … → v54 ≈95.996 → **v58 ≈96.12** |

**Note:** local-43 mean ≠ online-183 mean; v63 is the local best and was **not** the uploaded online package. Details: [SCORES.md](SCORES.md).

<p align="center">
  <img src="assets/score_chain_guikong.png" alt="Online score curve" width="720"/>
</p>
<p align="center">
  <img src="assets/score_chain_roles.png" alt="Submission types" width="720"/>
</p>

---

## 📋 Task & dataset

| Item | Content |
|------|------|
| **Competition** | [Pazhou × Intelligent Driving](https://www.aicompetition-pz.com/topic_detail/56) |
| **Task** | Output steering / throttle / brake from perception + pose / nav / dynamics at unsignalized intersections with dynamic interaction |
| **Score** | Safety 50% + efficiency 30% + path completion 20% |
| **Dataset (official)** | Pose, nav, dynamics, perception; unstructured unsignalized intersections; same schema prelim/finals, different distribution |
| **Public pack** | 2 maps (Dapai / Jiangtong) · **40** public scenes · baseline + docs; online ~**183** scenes |
| **Demo 11** | Dapai `1_3_10/1_3_15/2_3_5/3_2_2` · Jiangtong `1_5_3/1_5_4/5_4_1/6_2_5/6_3_3/9_3_2/9_3_3` |

Full write-up: [TASK_AND_DATA_en.md](TASK_AND_DATA_en.md) · Chinese [TASK_AND_DATA.md](TASK_AND_DATA.md). Official data is **not** shipped here.

---

## 🎥 Demo

Two render views; same verified v63 trajectories (post-process does not change sim).

| View | What you see | Full MP4s |
|------|------|----------|
| **Follow-cam** | Ego lock · smooth tracking · 1080² | [Release `v63-demo-videos`](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/tag/v63-demo-videos) (~144MB) |
| **Wide / map** | 1500² panorama — map, agents, full route | [Release `v63-demo-videos-wide`](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/tag/v63-demo-videos-wide) (`wide_*`, ~55MB) |

### Follow-cam preview

<p align="center">
  <img src="assets/preview_follow_2x2.gif" alt="follow-cam 2x2 preview" width="720">
</p>
<p align="center"><sub>2×2: Dapai 1_3_10 · Dapai 3_2_2 · Jiangtong 1_5_4 · Jiangtong 6_3_3</sub></p>

Full highlight reel: [`v63_highlights_1080.mp4`](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos/v63_highlights_1080.mp4)

### Wide / map preview

<p align="center">
  <img src="assets/preview_wide_2x2.gif" alt="wide/map 2x2 preview" width="720">
</p>
<p align="center"><sub>2×2: Dapai 1_3_10 · Jiangtong 1_5_4 · Jiangtong 6_3_3 · wide highlights clip</sub></p>

Full wide highlights: [`wide_v63_highlights_1080.mp4`](https://github.com/cainiao33/MineSim-MPC-AD-Control/releases/download/v63-demo-videos-wide/wide_v63_highlights_1080.mp4)

Per-scene 1x / 0.5x links for all 11 scenes are in the Chinese [README.md](README.md#成片清单11-场--双倍速--双视角).

---

## ⚠️ Reproduction notes

- This repo **already includes** mine_sim V1.1 + strategy patch + public 40 scenes + both mine maps — **clone once** to reproduce public runs
- Online hidden scenes and the platform judge are **out of tree**
- Absolute machine paths were removed from `planner.py` / `simple_control.py` / `sas_hook.py` — see [MANIFEST.json](MANIFEST.json)
- Optional: if you keep a separate official tree, use `scripts/overlay_into_minesim.sh` to copy strategy files only

---

## 📦 Install

```bash
git clone https://github.com/cainiao33/MineSim-MPC-AD-Control.git
cd MineSim-MPC-AD-Control
pip install -r setup/requirements.txt
python devkit/__main__.py \
  --scenario Scenario-dapai_intersection_1_3_10.json \
  --visualize none
```

| Env var | Role |
|------|------|
| `MINE_SCEN_CONFIG` / `MINE_CFG` | Explicit `scen_config.json` |
| `MINE_CENTERLINE_DIR` | Centerline directory |
| `MINE_EXPECTED_JSON` | Optional expected-time table |

Default: `devkit/centerlines/scen_config.json`.

---

## 🧠 Method in brief

1. **Replay timing:** non-reactive background agents; careless ego retiming/reroute can create conflicts.  
2. **Three-layer scene-adaptive config:** path → speed/IDM/yield → lateral-accel/steering; single-variable A/B with rollback.  
3. **Pipeline:** SAS closed-loop search → STPP/retiming (v58) → v63 full-trace SQP + goal refinement.

| Doc | Content |
|------|------|
| [TASK_AND_DATA_en.md](TASK_AND_DATA_en.md) | Task, scoring, dataset, public scenes |
| [METHOD.md](METHOD.md) | Method write-up |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Module map + config discovery |
| [SCORES.md](SCORES.md) | Online vs local scores |
| [MANIFEST.json](MANIFEST.json) | Per-file sha256 (58 strategy files) |

Chinese README: [README.md](README.md).

---

## 📄 License & upstream

- Upstream simulator: MineSim / mine_sim (**AGPL-3.0**) — see [UPSTREAM_README.md](UPSTREAM_README.md)
- This full tree (simulator + strategy) is AGPL-3.0 — [LICENSE](LICENSE) / [NOTICE](NOTICE)
