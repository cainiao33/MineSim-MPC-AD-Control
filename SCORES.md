# Scores

Keep **online 183** and **local 40/43** separate. Do not substitute one for the other.

## Headline numbers

| Scope | Version | Score | Note |
|---|---|---:|---|
| Online (183 scenes) | **v58-full-ff** | **96.121366…** (display ≈96.12) | Bindable submitted Docker lineage |
| Local public 40 | **v63** | **97.572355** | Official local scorer |
| Local known 43 (40+3 hidden) | **v63** | **97.096258…** | Zero vehicle / road-boundary collisions on that set |
| Leaderboard snapshot (team) | — | ≈96.1252 (4th at time of note) | Snapshot ≠ package SHA binding |

v63 was **not** packaged/uploaded as the online certification artifact in the pause handoff; v58 remains the cited online baseline.

## Early public-set ladder (local, historical)

Illustrative local progression on the public development set (not 183-mean):

`baseline ~74 → … → oracle+vprof ~89 → fin2 ~90.23`

Then platform first score **82.00** on 183 scenes established the hidden-set gap that later submissions closed with scene-adaptive banks and code axes.

## Online submission chain (selected milestones)

Abbreviated; full contest ledger lived in the team workspace.

| Milestone | Online ≈ | Theme |
|---|---:|---|
| v1 | 82.00 | First platform score |
| v6 | 87.91 | Winner merge (mode/vel0/a_max) |
| v11 | 89.39 | Yield hold-to-timeout |
| v20 | 90.27 | First break of 90 (rot axis) |
| v25 | 90.47 | alat bank + paid probes |
| v32 | 90.61 | Exact-future timing (narrow takeover set) |
| v41–v43 | 92.24→92.40 | SAS v2 platform wins + budget/saturation |
| v54 | 95.995956 | STPP geometry lineage |
| **v58** | **96.121366** | Full feed-forward / retiming certification |

Record breakers and paid probes both appear on the role chart:

![score chain](assets/score_chain_guikong.png)

![roles](assets/score_chain_roles.png)

## Residual gap (context only)

From v58 toward a 99 target, remaining mass concentrates in a few map families (efficiency + safety loss), not a single public scene. Local 43 alignment with online on shared scenes was verified for v57/v58 pairs; unknown hidden scenes stay unknown without platform reads.
