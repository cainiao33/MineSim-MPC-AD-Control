"""安全闭环参考的曲率事件锚点研究；XY单位米、曲率单位1/m。

道路事件决定锚点，首末切向约束的 C2 三次样条决定连接；不搜索均匀
段数或 Bezier 柄长。锚点本身不保证道路安全，仍用官方真实足迹筛选，
并由调用方从真实当前状态进行完整同码动态 rollout。
"""

import math
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.signal import savgol_filter


def _position(stations, ss, points):
    return np.column_stack([np.interp(stations, ss, points[:, axis])
                            for axis in (0, 1)])


def _dominant_turn(heading):
    """最大有序航向增量/减量；允许净转角小的 S 弯有主转弯事件。"""
    low = high = 0
    best = (0., 0, 0, 1.)
    for index in range(1, len(heading)):
        rise = float(heading[index] - heading[low])
        fall = float(heading[high] - heading[index])
        if rise > best[0]:
            best = (rise, low, index, 1.)
        if fall > best[0]:
            best = (fall, high, index, -1.)
        if heading[index] < heading[low]:
            low = index
        if heading[index] > heading[high]:
            high = index
    return best


def _knots(length, stations):
    """相邻事件至少4m；首末节点固定，不生成短段参数网格。"""
    selected = [0.]
    for station in sorted(set(float(x) for x in stations)):
        if station >= selected[-1] + 4. and station <= length - 4.:
            selected.append(station)
    selected.append(float(length))
    return np.asarray(selected)


def _event_templates(ss, points):
    grid = np.linspace(0., ss[-1], max(41, int(math.ceil(ss[-1] / .5)) + 1))
    xy = _position(grid, ss, points)
    spacing = float(grid[1] - grid[0])
    window = min(15, (len(grid) // 2) * 2 - 1)
    dx = savgol_filter(xy[:, 0], window, 3, deriv=1, delta=spacing)
    dy = savgol_filter(xy[:, 1], window, 3, deriv=1, delta=spacing)
    heading = np.unwrap(np.arctan2(dy, dx))
    # 空间平滑只用于事件检测；不会把滤波后的位置直接当成新路径。
    heading = savgol_filter(heading, window, 3)
    curvature = np.gradient(heading, grid)
    excursion, begin, end, side = _dominant_turn(heading)
    if excursion < .35 or grid[end] - grid[begin] < 12.:
        return [], dict(reason="no_significant_turn", turn_rad=excursion)

    region = np.arange(begin, end + 1)
    signed = side * curvature[region]
    peak = int(region[int(np.argmax(signed))])
    peak_curvature = float(max(0., side * curvature[peak]))
    if peak_curvature < .002:
        return [], dict(reason="no_curvature_core", turn_rad=excursion)
    threshold = max(.002, .25 * peak_curvature)
    left = right = peak
    while left > begin and side * curvature[left - 1] >= threshold:
        left -= 1
    while right < end and side * curvature[right + 1] >= threshold:
        right += 1
    # 孤立尖峰至少给8m支撑，避免仅围绕一两个采样点放节点。
    core_left = max(float(grid[begin]), min(float(grid[left]), float(grid[peak]) - 4.))
    core_right = min(float(grid[end]), max(float(grid[right]), float(grid[peak]) + 4.))
    target_heading = heading[begin] + side * excursion / 2.
    half = int(region[int(np.argmin(np.abs(heading[region] - target_heading)))])

    # 主转弯之前若有显著反向弯，保留其航向极值作为道路分支约束。
    reversal = (grid[begin] >= 8.
                and side * float(heading[begin] - heading[0]) <= -.15)
    common = [float(grid[begin])] if reversal else []
    # 三个结构模板分别约束主转弯角中点、曲率区边界、曲率峰；没有
    # 段数/柄长笛卡尔积，重复的事件节点及几何后续直接去重。
    templates = [
        ("EV_REV_HALF", common + [float(grid[half])]),
        ("EV_CORE_HALF", common + [core_left, float(grid[half]), core_right]),
        ("EV_CORE_PEAK", common + [core_left, float(grid[peak]), core_right]),
    ]
    detail = dict(turn_rad=excursion, turn_side=side,
                  turn_begin_m=float(grid[begin]), turn_end_m=float(grid[end]),
                  reversal_anchor_m=float(grid[begin]) if reversal else None,
                  half_turn_m=float(grid[half]), peak_curvature_m=float(grid[peak]),
                  core_begin_m=core_left, core_end_m=core_right,
                  reference_peak_curvature_1pm=peak_curvature)
    return templates, detail


def event_candidates(ctx, reference, t, datasets_dir, coefficients=(.85, 1.), looks=(6.,)):
    """最多3条事件样条×现有2政策；返回 (TraceCand列表, 静态审计)。

    输入路径仅来自当前安全到达 rollout 的 recs；无场名条件、外部 CSV
    或离线终点姿态。C2 指位置对样条参数的一二阶导数连续；非零切向时
    几何曲率亦连续，实际侧向加速度仍受车辆跟踪瞬态影响。
    """
    from trajectory_candidates import TraceCand
    import score_embed
    from score_result import road_boundary_collision

    if len(coefficients) > 2 or len(looks) > 1:
        raise ValueError("事件研究只允许现有两档速度和一个预瞄，动态上界为6")
    if reference["end"] != 4:
        return [], []
    start = np.asarray(ctx.prefix[-1][1:3], dtype=float)
    points = np.asarray([start] + [list(p) for _, p in sorted(reference["recs"].items())])
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > .03]
    points = points[keep]
    if len(points) < 5:
        return [], []
    ss = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    length = float(ss[-1])
    if not (20. <= length <= 350.) or not np.isfinite(points).all():
        return [], []
    templates, events = _event_templates(ss, points)
    if not templates:
        return [], []
    yaw = float(ctx.prefix[-1][3])
    start_direction = np.array([math.cos(yaw), math.sin(yaw)])
    previous = _position([max(0., length - 3.)], ss, points)[0]
    end_direction = points[-1] - previous
    end_direction /= max(1e-6, float(np.linalg.norm(end_direction)))
    shape = ctx.ego_shape
    actual_shape = dict(width=shape["width"], length=shape["length"],
                        head=shape["locationPoint2Head"], rear=shape["locationPoint2Rear"])
    score_embed.install_map_cache()
    sample_s = np.linspace(0., length, max(100, int(math.ceil(length * 4.)) + 1))
    seen_knots, seen_paths = set(), []
    candidates, audit = [], []
    for source, anchors in templates:
        knots = _knots(length, anchors)
        token = tuple(np.round(knots, 6))
        if token in seen_knots:
            continue
        seen_knots.add(token)
        knot_xy = _position(knots, ss, points)
        record = dict(source=source, reference_m=length, knots_m=knots.tolist(),
                      knot_xy=knot_xy.tolist(), events=events, construction="clamped_c2_cubic_spline")
        spline = CubicSpline(knots, knot_xy, axis=0,
                             bc_type=((1, start_direction), (1, end_direction)))
        xy = spline(sample_s)
        velocity, acceleration = spline(sample_s, 1), spline(sample_s, 2)
        norm = np.linalg.norm(velocity, axis=1)
        if not np.isfinite(xy).all() or not np.isfinite(norm).all() or float(norm.min()) < .1:
            audit.append(dict(record, accepted=False, reason="degenerate_tangent"))
            continue
        if any(np.allclose(xy, old, atol=.02, rtol=0.) for old in seen_paths):
            continue
        seen_paths.append(xy)
        headings = np.unwrap(np.arctan2(velocity[:, 1], velocity[:, 0]))
        curvature = ((velocity[:, 0] * acceleration[:, 1] - velocity[:, 1] * acceleration[:, 0])
                     / norm ** 3)
        reached = np.flatnonzero([ctx._in_goal(x, y) for x, y in xy])
        if not len(reached):
            audit.append(dict(record, accepted=False, reason="no_goal"))
            continue
        limit = int(reached[0]) + 1
        indices = np.arange(0, limit, 2)
        if indices[-1] != limit - 1:
            indices = np.r_[indices, limit - 1]
        geometry = xy[indices]
        frame = score_embed.build_frame(
            np.arange(len(geometry)) * ctx.dt,
            dict(x=geometry[:, 0], y=geometry[:, 1], yaw=headings[indices],
                 v=np.ones(len(geometry))), terminal=4, ego_shape=actual_shape)
        hit, _ = road_boundary_collision(frame, Path(f"{ctx.name}_result.csv"), datasets_dir)
        distances = np.linalg.norm(np.diff(xy[:limit], axis=0), axis=1)
        speed_cap = np.minimum(12., np.sqrt(.39 / np.maximum(np.abs(curvature[:limit]), 1e-5)))
        geometric_time = float(np.sum(distances / np.minimum(speed_cap[:-1], speed_cap[1:])))
        audit.append(dict(record, accepted=not hit, reason="footprint" if hit else "ok",
                          proposed_m=float(distances.sum()), geometry_time_s=geometric_time,
                          minimum_parametric_speed=float(norm.min()),
                          max_curvature_1pm=float(np.max(np.abs(curvature[:limit])))))
        if not hit:
            candidates.extend(TraceCand(xy, coefficient, look, t, source=source)
                              for coefficient in coefficients for look in looks)
    return candidates, audit
