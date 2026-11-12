"""仅由当前姿态、目标区和安全rollout构造曲线候选；XY为地图米制坐标。"""

import math
from pathlib import Path

import numpy as np


def bezier_candidates(ctx, reference, t, datasets_dir, coefficients=(.85, 1.),
                      looks=(6.,), delays=(0.,), scales=(.35, .65, 1.)):
    """从安全到达参考提取末端方向，搜索九条三次Bezier捷径。

    没有场名条件和外部参考。官方真实车身足迹先静态预筛；返回的候选
    仍必须从真实当前状态进行同码动态rollout，静态通过不代表动态安全。
    """
    from trajectory_candidates import TraceCand
    import score_embed
    from score_result import road_boundary_collision

    if reference["end"] != 4:
        return [], []
    start = np.asarray(ctx.prefix[-1][1:3], dtype=float)
    points = np.asarray([start] + [list(p) for _, p in sorted(reference["recs"].items())])
    ds = np.linalg.norm(np.diff(points, axis=0), axis=1)
    ss = np.r_[0., np.cumsum(ds)]
    length = float(ss[-1])
    if not (20. <= length <= 350.):
        return [], []
    end = points[-1]
    previous = np.array([np.interp(max(0., length - 3.), ss, points[:, k]) for k in (0, 1)])
    direction = end - previous
    direction /= max(1e-6, np.linalg.norm(direction))
    yaw = float(ctx.prefix[-1][3])
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    shape = ctx.ego_shape
    actual_shape = dict(width=shape["width"], length=shape["length"],
                        head=shape["locationPoint2Head"], rear=shape["locationPoint2Rear"])
    score_embed.install_map_cache()
    u = np.linspace(0., 1., max(100, int(length * 4.)))[:, None]
    out, audit = [], []
    for a in scales:
        for b in scales:
            p1, p2 = start + a * length * forward, end - b * length * direction
            xy = ((1. - u) ** 3 * start + 3. * (1. - u) ** 2 * u * p1
                  + 3. * (1. - u) * u ** 2 * p2 + u ** 3 * end)
            tangent = (3. * (1. - u) ** 2 * (p1 - start)
                       + 6. * (1. - u) * u * (p2 - p1)
                       + 3. * u ** 2 * (end - p2))
            yaw_array = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
            reached = np.flatnonzero([ctx._in_goal(x, y) for x, y in xy])
            if not len(reached):
                audit.append(dict(a=a, b=b, accepted=False, reason="no_goal"))
                continue
            limit = int(reached[0]) + 1
            geometry = xy[:limit:2]
            frame = score_embed.build_frame(
                np.arange(len(geometry)) * ctx.dt,
                dict(x=geometry[:, 0], y=geometry[:, 1], yaw=yaw_array[:limit:2],
                     v=np.ones(len(geometry))), terminal=4, ego_shape=actual_shape)
            hit, _ = road_boundary_collision(frame, Path(f"{ctx.name}_result.csv"), datasets_dir)
            new_length = float(np.linalg.norm(np.diff(xy[:limit], axis=0), axis=1).sum())
            audit.append(dict(a=a, b=b, accepted=not hit, reason="footprint" if hit else "ok",
                              reference_m=length, proposed_m=new_length))
            if hit:
                continue
            source = f"BZ_A{a:g}_B{b:g}"
            out.extend(TraceCand(xy, coefficient, look, t, delay=delay, source=source)
                       for coefficient in coefficients for look in looks for delay in delays)
    return out, audit
