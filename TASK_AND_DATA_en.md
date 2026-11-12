# Task Scenarios & Planning/Control Dataset

> Source: official statement for the [Pazhou Algorithm Competition × Intelligent Driving Ecosystem Track](https://deepvision.aicompetition-pz.com/#/homeDetail?id=2095413633562579532), plus local inspection of the public `mine_sim` package. This repo does **not** redistribute official data.

## 1. Task (planning / decision / control)

Given perception results, ego pose, navigation commands, and vehicle dynamics, output executable steering / throttle / brake in complex roads including **dynamic interaction** and **unsignalized intersections**.

| Item | Detail |
|------|------|
| Competition | [Pazhou × Intelligent Driving](https://deepvision.aicompetition-pz.com/#/homeDetail?id=2095413633562579532) |
| Site | https://www.aicompetition-pz.com/ |
| Track | Planning / decision / control (Preliminary B / Final B) |
| Simulator | MineSim / mine_sim (replay; background agents follow reference trajectories) |
| Contestant API | `devkit/planner.py` · `predictor.py` · `simple_control.py` |
| Submission | Docker image; max 3 uploads / day |
| Overall weight | Perception 50% + planning/control 50% (preliminary) |

### Scoring (official)

| Term | Weight | Focus |
|------|------|----------|
| Safety | 50% | Vehicle / boundary collisions, lateral clearance, lateral accel, TTC, roll/pitch/overturn, … |
| Efficiency | 30% | Completion, time, arrival heading error, compute, … |
| Path completion | 20% | Waypoints / valid goal arrival |

Finals also stress uncertain traffic, boundary / slope / surface changes (normal vs complex scenes scored separately).

## 2. Dataset (official description)

Preliminary / finals: **same field schema, different distribution**.

- Ego pose sequences
- Navigation commands
- Dynamics (speed, acceleration, yaw rate, …)
- Associated perception results
- Unsignalized intersections on unstructured roads
- Includes a baseline and documentation

Obtain data via official contest channels only. This repo only ships a strategy code patch.

## 3. Public package (local summary)

| Asset | Content |
|------|------|
| Maps | **Guangdong Dapai** · **Jiangxi Jiangtong** |
| Public scenes | **40** intersection JSONs + `expected_times.json` |
| Online eval | ~**183** scenes (40 public + hidden); mean of per-scene totals |
| Baseline | Reference-line planning · trajectory prediction · IDM + pure pursuit |

### Demo 11 scenes (videos in this repo)

All arrive with zero collision / curb hit (consistent with verified v63 behavior):

| Map | IDs |
|------|---------|
| Dapai | `1_3_10` · `1_3_15` · `2_3_5` · `3_2_2` |
| Jiangtong | `1_5_3` · `1_5_4` · `5_4_1` · `6_2_5` · `6_3_3` · `9_3_2` · `9_3_3` |

## 4. Scenario mechanics

- Replay, not reactive multi-agent game: ego retiming/reroute can create self-inflicted conflicts.
- Scorer footprint checks are often stricter than simulator termination.
- Efficiency is not “faster is always better”; time score caps versus expected duration.

See [METHOD.md](METHOD.md) and [SCORES.md](SCORES.md). Chinese: [TASK_AND_DATA.md](TASK_AND_DATA.md).
