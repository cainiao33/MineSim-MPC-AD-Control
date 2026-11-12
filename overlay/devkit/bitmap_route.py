"""有界姿态格点前向路线研究；地图米制XY，航向弧度。

目标区域不约束终端航向，搜索无需先有到达参考。位图点采样仅用于
提案；最终Trace参考必须经过官方完整车身足迹检查，动态安全另行验证。
"""

import heapq
import math
from pathlib import Path
import time

import numpy as np


def build_candidates(ctx, reference, incumbent, datasets_dir, budget_s=8.):
    build_started = time.perf_counter()
    from trajectory_candidates import TraceCand
    import score_embed
    from score_result import road_boundary_collision

    start = np.asarray(ctx.prefix[-1][1:4], dtype=float)
    vertices = np.column_stack([ctx.goal["x"], ctx.goal["y"]])
    target = vertices.mean(axis=0)
    shape = ctx.ego_shape
    # 直接使用车辆声明的最小转弯半径，避免靠高曲率折线伪造捷径。
    radius = max(12., float(shape.get("min_turn_radius", 14.2)))
    xy_ref = np.asarray([start[:2]] + [list(p) for _, p in
                        sorted(reference.get("recs", {}).items())])
    # 未达参考可能只包含部分绕行；保留银行静态路线作为搜索范围依据。
    # 只读取XY，不改ctx.path，也不把旧路线当新政策。
    path_points = np.asarray([[point[0], point[1]] for point in ctx.path], dtype=float)
    bound_points = np.vstack([xy_ref, path_points.reshape(-1, 2), vertices])
    lower, upper = bound_points.min(axis=0) - 20., bound_points.max(axis=0) + 20.
    _, image, x0, y0 = ctx._road_kit()
    # 分辨率由地图metadata读取（当前10px/m）；最终门仍走官方足迹评分器。
    import json
    location = "guangdong_dapai" if ctx.name.split("_", 1)[0] == "dapai" else "jiangxi_jiangtong"
    map_info = json.loads((Path(datasets_dir) / "maps/semantic_map" /
                           f"{location}_semantic_map.json").read_text())["bitmap_mask_PNG"]
    scale = float(map_info["scale_PixelPerMeter"])
    long = np.linspace(-float(shape["locationPoint2Rear"]) - .3,
                       float(shape["locationPoint2Head"]) + .3,
                       int(math.ceil(float(shape["length"]))) + 2)
    lat = np.linspace(-float(shape["width"]) / 2. - .3,
                      float(shape["width"]) / 2. + .3,
                      int(math.ceil(float(shape["width"]))) + 2)
    longitudinal, lateral = (a.ravel() for a in np.meshgrid(long, lat))

    def footprint_safe(states):
        cosine, sine = np.cos(states[:, 2, None]), np.sin(states[:, 2, None])
        xs = states[:, 0, None] + cosine * longitudinal - sine * lateral
        ys = states[:, 1, None] + sine * longitudinal + cosine * lateral
        ix, iy = ((xs - x0) * scale).astype(int), ((ys - y0) * scale).astype(int)
        if np.any(ix < 0) or np.any(ix >= image.shape[1]) or np.any(iy < 0) or np.any(iy >= image.shape[0]):
            return False
        return bool(image[iy, ix].all())

    distances = np.arange(.5, 4.01, .5)
    curvatures = (0., 1. / radius, -1. / radius)

    def advance(state, curvature):
        x, y, yaw = state
        if curvature == 0.:
            return np.column_stack([x + distances * math.cos(yaw),
                                    y + distances * math.sin(yaw),
                                    np.full(len(distances), yaw)])
        headings = yaw + curvature * distances
        return np.column_stack([x + (np.sin(headings) - math.sin(yaw)) / curvature,
                                y - (np.cos(headings) - math.cos(yaw)) / curvature,
                                headings])

    def key(state):
        return (int(round(state[0])), int(round(state[1])),
                int(round((state[2] % (2. * math.pi)) / (math.pi / 18.))) % 36)

    def heuristic(state):
        # 加权几何启发仅用于有界提案，无全局最优/下界声明。
        delta = np.maximum(0., np.maximum(vertices.min(axis=0) - state[:2],
                                         state[:2] - vertices.max(axis=0)))
        return float(np.linalg.norm(delta)) / 4.

    started = time.perf_counter()
    nodes = [(start, -1, None, 0., 0.)]
    heap = [(heuristic(start), 0)]
    costs = {key(start): 0.}
    expanded = 0
    goal_paths = []
    while heap and expanded < 12000 and time.perf_counter() - started < budget_s:
        _, index = heapq.heappop(heap)
        state, parent, edge, cost, length = nodes[index]
        if cost > costs.get(key(state), math.inf) + 1e-9:
            continue
        expanded += 1
        for curvature in curvatures:
            states = advance(state, curvature)
            if np.any(states[:, :2] < lower) or np.any(states[:, :2] > upper):
                continue
            if not footprint_safe(states):
                continue
            end = states[-1]
            speed = 12. if curvature == 0 else .85 * math.sqrt(.39 / abs(curvature))
            new_cost = cost + 4. / speed
            new_length = length + 4.
            if new_length > 350.:
                continue
            inside = [i for i, pose in enumerate(states) if ctx._in_goal(*pose[:2])]
            if inside:
                portions = [states[:inside[0] + 1, :2]]
                prior = index
                while prior >= 0:
                    old = nodes[prior]
                    if old[2] is not None:
                        portions.append(old[2][:, :2])
                    prior = old[1]
                points = np.vstack([start[None, :2], *reversed(portions)])
                # 保存两个不同终端方向的路线，不铺速度/半径参数网格。
                terminal_yaw = states[inside[0], 2]
                if any(abs((terminal_yaw - yaw + math.pi) % (2. * math.pi) - math.pi) < .4
                       for _, yaw, _ in goal_paths):
                    continue
                goal_paths.append((points, terminal_yaw, new_cost))
                if len(goal_paths) >= 2:
                    heap.clear()
                    break
                continue
            encoded = key(end)
            if new_cost >= costs.get(encoded, math.inf) - 1e-9:
                continue
            costs[encoded] = new_cost
            nodes.append((end, index, states, new_cost, new_length))
            heapq.heappush(heap, (new_cost + heuristic(end), len(nodes) - 1))
    search_elapsed = time.perf_counter() - started
    score_embed.install_map_cache()
    actual_shape = dict(width=shape["width"], length=shape["length"],
                        head=shape["locationPoint2Head"], rear=shape["locationPoint2Rear"])
    candidates, records = [], []
    for index, (points, terminal_yaw, cost) in enumerate(goal_paths):
        candidate = TraceCand(points, .85, 6., ctx.prefix[-1][0],
                              source=f"BITMAP_ROUTE{index}")
        path = np.column_stack([candidate.rx, candidate.ry])
        hits = np.flatnonzero([ctx._in_goal(x, y) for x, y in path])
        if not len(hits):
            records.append(dict(reason="resampling_misses_goal", accepted=False))
            continue
        limit = int(hits[0]) + 1
        frame = score_embed.build_frame(np.arange(limit) * ctx.dt,
            dict(x=path[:limit, 0], y=path[:limit, 1], yaw=candidate.heading[:limit],
                 v=np.ones(limit)), terminal=4, ego_shape=actual_shape)
        hit, check_kind = road_boundary_collision(
            frame, Path(f"{ctx.name}_result.csv"), datasets_dir)
        accepted = not hit and check_kind == "bitmap_footprint"
        reason = "official_footprint" if hit else (
            "ok" if accepted else "missing_official_footprint")
        records.append(dict(candidate=candidate.label, accepted=accepted,
                            reason=reason, road_boundary_check=check_kind,
                            search_time_cost_s=cost, terminal_yaw=terminal_yaw,
                            path_xy=path.tolist(), final_trace_heading=candidate.heading.tolist()))
        if accepted:
            candidates.append(candidate)
    return candidates, dict(construction="bounded_forward_pose_bitmap_search",
                            candidate_count=len(candidates), expanded=expanded,
                            node_count=len(nodes), reached_route_count=len(goal_paths),
                            minimum_turn_radius_m=radius,
                            elapsed_s=time.perf_counter() - build_started,
                            search_elapsed_s=search_elapsed,
                            max_expanded=12000, budget_s=budget_s,
                            heuristic_optimality_claimed=False,
                            raw_bitmap_sampling_is_final_safety=False, records=records)
