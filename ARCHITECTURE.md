# Architecture

## Runtime loop (official mine_sim)

```
observation
  → Predictor.predict
  → MotionController / simple_control.process
  → action (acc, steering)
```

This overlay replaces/extends the contestant-facing `devkit` pieces while leaving the Env, maps, and scorer to the official tree.

## Overlay map

```
overlay/devkit/
  planner.py              # path modes + centerline / v2s table selection
  predictor.py            # prediction + SAS stash hooks
  simple_control.py       # tracking, IDM/alat/rot/look/yield, SAS/STPP entry
  centerlines/
    scen_config.json      # 183-scene adaptive bank (full)
    centerlines_jiangxi_jiangtong_v2s.json
  sas_core.py / sas_hook.py
  stpp_*.py / executed_retiming*.py
  control_full_trace_*.py / control_live_goal_polygon.py / ...
  bitmap_route.py / bootstrap_route.py / geometry_*.py / ...
```

Stock centerline tables (`centerlines_guangdong_dapai.json`, `centerlines_jiangxi_jiangtong.json`) remain those shipped with official mine_sim; only the **v2s** table and `scen_config.json` are overlaid.

## Module groups

| Group | Files (representative) | Job |
|---|---|---|
| Path bank | `planner.py`, `centerlines/*`, `bitmap_route.py`, `bootstrap_route.py` | Scene mode + corridor geometry |
| Tracking bank | `simple_control.py`, `predictor.py` | Speed/safety scene keys + control |
| SAS | `sas_hook.py`, `sas_core.py`, `score_embed.py` | Closed-loop candidate search |
| STPP / retiming | `stpp_hook.py`, `stpp_geometry.py`, `executed_retiming*.py` | Executed-geometry timing |
| Full-trace SQP | `control_full_trace_*.py`, `full_trace_*.py`, `control_live_goal_polygon.py` | v63 goal-aware refinement |
| Trials / meters | `*_trials.py`, `*_work_meter.py` | Search budgets and admission |

## Config discovery (portable)

After open-source path scrubbing, config search order is roughly:

1. `MINE_CFG` / `MINE_SCEN_CONFIG` (explicit file)
2. `devkit/centerlines/scen_config.json` (next to the code)
3. `./outputs/scen_config.json` (CWD fallback)

Centerlines: `MINE_CENTERLINE_DIR` → `devkit/centerlines/` → `./outputs/`.

Optional expected times: `MINE_EXPECTED_JSON` → `<mine_sim>/inputs/expected_times.json`.

## Install shape

```bash
bash scripts/overlay_into_minesim.sh /path/to/official_mine_sim
```

Copies only NEW+MOD files into `devkit/`, backs up overwritten stock files once under `.mpc_ad_overlay_backup/`.
