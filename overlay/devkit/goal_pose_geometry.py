"""目标区入口位置和到达朝向联合搜索；XY为地图米制坐标。"""

import math
from pathlib import Path

import numpy as np


def goal_pose_candidates(ctx, reference, t, datasets_dir, coefficients=(.85, 1.),
                         looks=(6.,), max_paths=6):
    """从起点姿态和目标多边形构建有界线段-圆弧-线段候选。

    每个目标采样点和末端方向解析解出圆弧半径，不继承参考唯一末端
    姿态。最多75个静态几何，按几何限速时间保留6条，再动态验证最多
    12个政策。静态足迹通过和几何时间都不代替同码闭环安全/评分。
    """
    from trajectory_candidates import TraceCand
    import score_embed
    from score_result import road_boundary_collision

    if reference["end"] != 4:
        return [], []
    start = np.asarray(ctx.prefix[-1][1:3], dtype=float)
    yaw = float(ctx.prefix[-1][3])
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    lateral = np.array([-forward[1], forward[0]])
    vertices = np.column_stack([ctx.goal["x"], ctx.goal["y"]])
    target = vertices.mean(axis=0)
    f, lat = float((target - start) @ forward), float((target - start) @ lateral)
    if not (12. <= abs(lat) <= 65. and abs(f) <= 40.):
        return [], []
    side = np.sign(lat)
    targets = np.vstack([target, target + .65 * (vertices - target)])
    shape = ctx.ego_shape
    actual_shape = dict(width=shape["width"], length=shape["length"],
                        head=shape["locationPoint2Head"], rear=shape["locationPoint2Rear"])
    score_embed.install_map_cache()
    safe, audit = [], []
    for goal_i, goal in enumerate(targets):
        goal_f, goal_lat = float((goal - start) @ forward), side * float((goal - start) @ lateral)
        for turn in (math.pi - .2, math.pi, math.pi + .2):
            cs, sn = math.cos(turn), math.sin(turn)
            for depth in (30., 40., 50., 60., 70.):
                radius = (sn * (goal_f - depth) - cs * goal_lat) / (1. - cs)
                record = dict(goal_i=goal_i, turn=turn, depth=depth, radius=radius)
                if not (8. <= radius <= 40.):
                    audit.append(dict(record, accepted=False, reason="radius"))
                    continue
                arc_f, arc_lat = depth + radius * sn, radius * (1. - cs)
                back_length = (goal_f - arc_f) * cs + (goal_lat - arc_lat) * sn
                if back_length < 0.:
                    audit.append(dict(record, accepted=False, reason="direction"))
                    continue
                line_s = np.arange(0., depth, .5)
                line = start + line_s[:, None] * forward
                theta = np.linspace(0., turn, max(30, int(turn * radius * 4.)))
                arc = (start + (depth + radius * np.sin(theta))[:, None] * forward
                       + side * (radius * (1. - np.cos(theta)))[:, None] * lateral)
                heading = cs * forward + side * sn * lateral
                back_s = np.arange(.5, back_length + 15., .5)
                back = arc[-1] + back_s[:, None] * heading
                xy = np.vstack([line, arc, back])
                headings = np.r_[np.full(len(line), yaw), yaw + side * theta,
                                 np.full(len(back), yaw + side * turn)]
                reached = np.flatnonzero([ctx._in_goal(x, y) for x, y in xy])
                if not len(reached):
                    audit.append(dict(record, accepted=False, reason="no_goal"))
                    continue
                limit = int(reached[0]) + 1
                geometry = xy[:limit:2]
                frame = score_embed.build_frame(
                    np.arange(len(geometry)) * ctx.dt,
                    dict(x=geometry[:, 0], y=geometry[:, 1], yaw=headings[:limit:2],
                         v=np.ones(len(geometry))), terminal=4, ego_shape=actual_shape)
                hit, _ = road_boundary_collision(frame, Path(f"{ctx.name}_result.csv"), datasets_dir)
                proposed_length = float(np.linalg.norm(np.diff(xy[:limit], axis=0), axis=1).sum())
                # 仅用于有界候选排序；交通、瞬态和评分仍由动态rollout决定。
                turn_length = radius * turn
                geometry_time = turn_length / min(12., math.sqrt(.39 * radius))
                geometry_time += max(0., proposed_length - turn_length) / 12.
                record = dict(record, proposed_m=proposed_length, geometry_time_s=geometry_time,
                              accepted=not hit, reason="footprint" if hit else "ok")
                audit.append(record)
                if not hit:
                    source = f"GP_G{goal_i}_A{turn - math.pi:.2f}_D{depth:g}"
                    safe.append((geometry_time, source, xy, record))
    out = []
    for _, source, xy, record in sorted(safe, key=lambda row: row[0])[:max_paths]:
        record["dynamic_selected"] = True
        out.extend(TraceCand(xy, coefficient, look, t, source=source)
                   for coefficient in coefficients for look in looks)
    return out, audit
