# Method

## One-sentence claim

We treat MineSim as a **non-reactive replay** benchmark and raise score with a closed loop of **scene-adaptive strategy selection + empirical bank/rollback**, not blind parameter search.

## Act 1 — Replay timing

Background agents follow reference trajectories in open loop. Ego speed changes or route changes can create self-inflicted conflict windows that a constant-speed centerline policy often avoids. Early local baseline ≈74 → constant-speed + scene oracle ≈90 on the public set established this physical reading before later mechanisms.

## Act 2 — Three layers

| Layer | Role | Main knobs |
|---|---|---|
| Path | Corridor / mode choice | `modes`: base / center / hybrid; offline centerlines (+ v2s table) |
| Speed | Timing vs replay sweep | `vel0`, curvature `vprof`, IDM (`idm_scenes`), yield hold |
| Safety / tracking | Cap harmful control | `alat_scenes`, `rot_scenes`, look-ahead, STPP geometry, SAS rollout |

Discipline: **single-variable A/B → family adjudication → winners enter the bank, losers roll back for free**. Platform aggregate equals the arithmetic mean of per-scene totals; local and platform matched bit-for-bit across many submission rounds.

## Act 3 — Later production stack (v58→v63)

On top of the banked scene config:

1. **SAS v2** — same-code closed-loop candidate rollouts with the official plant/controller/scorer; select the best admissible commit window.
2. **STPP geometry / retiming feed-forward** — reshape executed timing without abandoning the certified safety model.
3. **Full-trace SQP + goal polygon dispatch (v63)** — refine selected native-safe tables under the original nonlinear safety gates; keep original hysteresis (`HYST`) rather than forcing new policies.

Exact-future / GT-horizon research informed timing, but the open overlay ships the **v63 production modules** that run under the default contest environment (platform ships with no custom env required for the submitted Docker lineage).

## Act 4 — Negative results culture

Axes that died in public A/B (wrong dose, family non-transfer, wall-clock cap, route shortcuts that pierce replay corridors) stayed out of the bank. The score chain mixes record breakers and paid probes; both are part of the method.

## Config as method

`overlay/devkit/centerlines/scen_config.json` contains the full **183-scene** adaptive tables used in production research. Scene-name keys are the explicit interface of “scene-adaptive selection,” not a hidden side channel outside the official observation/API.

## What we do not claim

- Local known-43 mean is **not** the online 183-scene mean.
- v63 is the **local** production best; the bindable online baseline referenced here is **v58 ≈ 96.12**.
- This overlay does not redistribute official maps or hidden inputs.
