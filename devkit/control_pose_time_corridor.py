"""F16：strict-safe source为incumbent的有界可达姿态/时间图。

rows=(N,5)为绝对t/x/y/yaw/v，actions=(N-1,3)为t/命令/finalδ。
每条图边保存原P/clip/jerk及旧速植物的执行结果；整车道路、GT时间与
中央差分侧加在边入图前核验。仅提取一个完整终态，不经样条或跟踪器
改写。复用冻结F15纯数学/道路/记录/消费合同，不调用其模型或优化器。
"""

from dataclasses import dataclass
import heapq
import math
from pathlib import Path
import time

import numpy as np

from control_feasible_portal_turn import (
    FeasiblePortalRefusal, LocalFeasiblePortalTurnCand, _RoadClearance,
    _recorder_frame, _rows_and_frames, _source_contract, _vertices, track_step,
)


STATIC_BUDGET_S = 8.
FINAL_RESERVE_S = 1.5
MAX_EXPANDED = 1200
MAX_EDGE_PROPOSALS = 6000
MAX_SCALAR_STEPS = 30000
EDGE_STEPS = 5
MAX_CHAIN_STEPS = 430
STEERING_LIMIT = .6
ALAT_LIMIT = .391
TURN_THRESHOLD = .1
TURN_CONTINUATION_THRESHOLD = .05
SUSTAINED_FRAMES = 10
MIN_TURN_SWEEP = .7
PRETURN_LEAD_S = .5
PRIORITY_DISTANCE_WEIGHT = 4.
CORNER_COMMAND_ALAT = .38


class PoseTimeRefusal(FeasiblePortalRefusal):
    pass


class _SearchStop(Exception):
    pass


class _Budget:
    def __init__(self, seconds, started):
        self.seconds, self.started = float(seconds), started
        self.search_seconds = max(0., self.seconds-FINAL_RESERVE_S)

    def check(self, stage, search=False):
        elapsed = time.perf_counter()-self.started
        if elapsed >= self.seconds:
            raise PoseTimeRefusal('static_budget_' + stage)
        if search and elapsed >= self.search_seconds:
            raise _SearchStop('search_final_reserve_reached')


def _angle(value):
    return (value+math.pi) % (2.*math.pi)-math.pi


def preturn_anchor(rows, action_frames, vertices):
    """首个在线持续弯事件授权唯一native起点图；不裁掉转前道路。"""
    rows, _, steering = _rows_and_frames(rows, action_frames)
    _vertices(vertices)
    if np.any(np.abs(steering) > STEERING_LIMIT+1e-12):
        raise PoseTimeRefusal('source_steering_exceeds_original_point_six_domain')
    yaw = np.unwrap(rows[:, 3])
    for onset in range(len(steering)-SUSTAINED_FRAMES+1):
        values = steering[onset:onset+SUSTAINED_FRAMES]
        if (np.any(np.abs(values) < TURN_THRESHOLD)
                or not np.all(np.sign(values) == np.sign(values[0]))):
            continue
        finish = onset+SUSTAINED_FRAMES
        while (finish < len(steering) and abs(steering[finish]) >= TURN_CONTINUATION_THRESHOLD
               and np.sign(steering[finish]) == np.sign(values[0])):
            finish += 1
        sweep = float(yaw[finish]-yaw[onset])
        if abs(sweep) < MIN_TURN_SWEEP:
            continue
        eligible = np.flatnonzero((rows[:onset, 0] <= rows[onset, 0]-PRETURN_LEAD_S+1e-8)
                                  & (np.abs(steering[:onset]) < TURN_THRESHOLD))
        # lead只留作事件参考；图唯一root恒为当前prefix最后状态，不扫切点。
        reference = int(eligible[-1]) if len(eligible) else None
        index = 0
        return index, dict(reason='first_sustained_online_turn_authorizes_current_native_prefix_root',
            index=index, cut_time_s=float(rows[index, 0]), onset_index=onset,
            turning_end_index=finish, turn_sign=int(np.sign(values[0])),
            heading_change_rad=sweep, sustained_frames=finish-onset,
            lead_s=float(rows[onset, 0]-rows[index, 0]), anchor_zero_allowed=True,
            reference_lead_index=reference,
            reference_lead_s=None if reference is None else float(rows[onset, 0]-rows[reference, 0]),
            search_root_policy='unique_current_native_prefix_state',
            requires_point_six_saturation=False)
    return None, dict(reason='no_sustained_significant_online_turn')


class PoseTimeCorridorCand(LocalFeasiblePortalTurnCand):
    kind = 'F16_POSE_TIME_CORRIDOR'

    def __init__(self, base, actions, dt, model_rows):
        values = np.asarray(actions, dtype=float)
        if (values.ndim != 2 or values.shape[1] != 3
                or not np.isfinite(values).all()
                or np.any(np.abs(values[:, 2]) > STEERING_LIMIT+1e-12)):
            raise PoseTimeRefusal('final_action_exceeds_original_steering_domain')
        super().__init__(base, actions, dt, model_rows)
        # init中绑定新家族身份；之后继承sealed表和同帧七元token。
        object.__setattr__(self, 'label', 'POSE_TIME_CORRIDOR_I1_' + base.label)

    def steering(self, ctrl):
        delta = super().steering(ctrl)
        if abs(delta) > STEERING_LIMIT+1e-12:
            raise ValueError('prefix_or_table_steering_exceeds_original_point_six_domain')
        return delta


@dataclass(frozen=True)
class _Node:
    row: tuple
    last_a: float
    delta: float
    previous_row: tuple | None
    previous_omega: float | None
    parent: int
    edge_rows: tuple
    edge_actions: tuple
    chain_steps: int
    source_node: bool = False
    edge_accelerations: tuple = ()


def _node_key(node, start_time, dt):
    """仅pose近似剪枝；时间/v/a/中央差分父历史绝不混并。"""
    t, x, y, yaw, speed = node.row
    return (int(round((t-start_time)/dt)), float(t).hex(), int(round(x/.5)), int(round(y/.5)),
            int(round(_angle(yaw)/(math.pi/36.))), float(speed).hex(),
            float(node.last_a).hex(), float(node.delta).hex(),
            None if node.previous_omega is None else float(node.previous_omega).hex(),
            None if node.previous_row is None else tuple(float(v).hex() for v in node.previous_row))


def _rounded(row):
    return np.asarray([round(float(row[0]), 1)]+[round(float(v), 4) for v in row[1:]])


def _closed_alat(previous, current, following):
    """追加下一行才闭合当前中心帧；首行forward，终态另做one-sided。"""
    rows = np.asarray([current, following] if previous is None else [previous, current, following])
    index = 0 if previous is None else 1
    raw = abs(float(rows[index, 4])*float(np.gradient(np.unwrap(rows[:, 3]), rows[:, 0])[index]))
    rounded = np.vstack([_rounded(row) for row in rows])
    recorder = abs(float(rounded[index, 4])
                   *float(np.gradient(np.unwrap(rounded[:, 3]), rounded[:, 0])[index]))
    return raw <= ALAT_LIMIT and recorder <= .4, raw, recorder


def _terminal_alat(previous, current):
    rows = np.asarray([previous, current], dtype=float)
    omega = float(np.unwrap(rows[:, 3])[1]-np.unwrap(rows[:, 3])[0])/(rows[1, 0]-rows[0, 0])
    raw = abs(float(current[4])*omega)
    rounded = np.vstack([_rounded(row) for row in rows])
    recorder_omega = float(np.unwrap(rounded[:, 3])[1]-np.unwrap(rounded[:, 3])[0]) / (rounded[1, 0]-rounded[0, 0])
    recorder = abs(float(rounded[1, 4])*recorder_omega)
    return raw <= ALAT_LIMIT and recorder <= .4, raw, recorder


def _corners(x, y, yaw, width, head, rear):
    direction = np.array([math.cos(yaw), math.sin(yaw)])
    lateral = np.array([-direction[1], direction[0]])*width/2.
    front, back = np.array([x, y])+head*direction, np.array([x, y])-rear*direction
    return np.asarray([front+lateral, front-lateral, back-lateral, back+lateral])


def _traffic_at_pose(row, background, shape):
    """官方逐pose碰撞/TTC/侧距同数学；无完整total评分或未来推进。"""
    from shapely.geometry import Polygon
    _, x, y, yaw, speed = row
    ego = _corners(x, y, yaw, shape['width'], shape['head'], shape['rear'])
    polygon = Polygon(ego).convex_hull
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    lateral = np.array([-forward[1], forward[0]])
    ego_long, ego_lat = ego@forward, ego@lateral
    for values in background.values():
        other_shape = values['shape']
        other = _corners(float(values['x']), float(values['y']), float(values['yaw_rad']),
                         float(other_shape['width']), float(other_shape['locationPoint2Head']),
                         float(other_shape['locationPoint2Rear']))
        other_polygon = Polygon(other).convex_hull
        if polygon.intersects(other_polygon):
            return False, 'GT_full_polygon_collision'
        other_long, other_lat = other@forward, other@lateral
        longitudinal_overlap = min(ego_long.max(), other_long.max())-max(ego_long.min(), other_long.min())
        lateral_overlap = min(ego_lat.max(), other_lat.max())-max(ego_lat.min(), other_lat.min())
        gap = float(other_long.min()-ego_long.max())
        closing = float(speed)-float(values['v_mps'])*math.cos(float(values['yaw_rad'])-yaw)
        if gap >= 0. and lateral_overlap >= 0. and closing > 1e-6 and gap/closing < 3.:
            return False, 'GT_TTC_below_three'
        if longitudinal_overlap >= 0. and polygon.distance(other_polygon) < .8:
            return False, 'GT_lateral_distance_below_point_eight'
    return True, 'pose_safe'


def _prepare_road(ctx, all_rows, shape, datasets_dir, budget, audit):
    import score_embed
    import score_result
    budget.check('before_local_road')
    vertices = np.column_stack([ctx.goal['x'], ctx.goal['y']])
    xy = np.vstack([np.asarray(all_rows)[:, 1:3], vertices])
    radius = math.hypot(max(shape['head'], shape['rear']), shape['width']/2.)
    padding = 2.*radius+2.
    bounds = (float(xy[:, 0].min()-padding), float(xy[:, 1].min()-padding),
              float(xy[:, 0].max()+padding), float(xy[:, 1].max()+padding))
    budget.check('before_map_cache_install')
    score_embed.install_map_cache()
    budget.check('before_official_patch_load')
    patch = score_result.load_map_patch(Path(datasets_dir), Path(f'{ctx.name}_result.csv'), bounds)
    audit['map_patch_loads'] += 1
    budget.check('after_official_patch_load')
    if patch is None:
        raise PoseTimeRefusal('official_bitmap_local_patch_unavailable')
    road = _RoadClearance(*patch, shape)
    budget.check('after_local_road')
    audit['road_region'] = dict(bounds=list(bounds), extent=list(road.extent),
        pixels=int(road.image.size), actual_shape=shape, source_tube_only=False,
        certification_scope='official_native_point_one_second_whole_pose_states',
        continuous_swept_safety_claimed=False)
    return road


def _background_at_time(ctx, t, budget, audit, phase, search=False):
    """只复用同ctx/key3原生memo；miss才调用原_synth，无actor/时间改写。"""
    budget.check(phase+'_GT_memo_lookup', search=search)
    audit['GT_queries'] += 1
    key = str(np.around(t, 3))
    memo = getattr(ctx, '_memo', None)
    entry = memo.get(key) if isinstance(memo, dict) else None
    if entry is None:
        budget.check(phase+'_GT_synth', search=search)
        audit['GT_synth_calls'] += 1
        entry = ctx._synth(t)
        if isinstance(memo, dict):
            memo[key] = entry
    else:
        audit['GT_memo_hits'] += 1
    budget.check(phase+'_after_GT_background', search=search)
    return entry[0]


def _pose_safe(row, ctx, road, shape, budget, audit, cache, phase):
    search = phase == 'edge'
    budget.check(phase+'_pose', search=search)
    key = tuple(float(v).hex() for v in row)
    if key in cache:
        audit['pose_cache_hits'] += 1
        return cache[key]
    if len(cache) >= audit['max_query_cache_entries']:
        raise PoseTimeRefusal('query_cache_cap_reached')
    t, x, y, yaw, _ = map(float, row)
    audit[phase+'_pose_checks'] += 1
    budget.check(phase+'_runtime_lookup', search=search)
    audit['runtime_queries'] += 1
    if ctx._road_hit(x, y, yaw):
        result = (False, 'runtime_bitmap_collision')
    else:
        try:
            for pose in (row, _rounded(row)):
                budget.check(phase+'_actual_full_polygon', search=search)
                audit['actual_polygon_queries'] += 1
                # 此oracle碰黑才抛错；白像素margin可为负，不伪作连通tube证书。
                road.witness_margin(pose)
        except FeasiblePortalRefusal as error:
            if str(error).startswith('static_budget_'):
                raise
            result = (False, 'actual_full_polygon_' + str(error))
        else:
            budget.check(phase+'_GT_t_plus_dt', search=search)
            background = _background_at_time(ctx, t, budget, audit, phase, search=search)
            budget.check(phase+'_runtime_actor_collision', search=search)
            audit['runtime_actor_collision_queries'] += 1
            if ctx._collide(x, y, yaw, background, str(np.around(t, 3))):
                result = (False, 'GT_runtime_footprint_collision')
            else:
                budget.check(phase+'_raw_traffic', search=search)
                audit['traffic_pose_checks'] += 1
                result = _traffic_at_pose(row, background, shape)
                if result[0]:
                    # 官方recorder对actor/ego字段均round4；保守同时核验。
                    rounded_bg = {actor: dict(values, **{k: round(float(values[k]), 4)
                        for k in ('x', 'y', 'yaw_rad', 'v_mps')}) for actor, values in background.items()}
                    budget.check(phase+'_rounded_traffic', search=search)
                    audit['traffic_pose_checks'] += 1
                    result = _traffic_at_pose(_rounded(row), rounded_bg, shape)
    budget.check(phase+'_after_pose_queries', search=search)
    cache[key] = result
    return result


def _verify_source(rows, frames, prefix, ctx, road, shape, budget, audit, cache):
    budget.check('before_source_action_math')
    if audit['scalar_steps']+len(frames) > MAX_SCALAR_STEPS:
        raise PoseTimeRefusal('source_action_math_exceeds_scalar_step_cap')
    # 预留整段代数预算；早拒时此数不是声称已经执行了全部源帧。
    audit['scalar_steps'] += len(frames)
    audit['source_action_math_steps_reserved'] = len(frames)
    _source_contract(rows, frames, float(ctx.dt), shape['length'])
    budget.check('after_source_action_math')
    all_rows = np.vstack([prefix, rows])
    for i, row in enumerate(all_rows):
        previous = None if i == 0 else all_rows[i-1]
        if i+1 < len(all_rows):
            good, _, _ = _closed_alat(previous, row, all_rows[i+1])
        else:
            good, _, _ = _terminal_alat(all_rows[i-1], row)
        if not good:
            raise PoseTimeRefusal('source_original_or_recorder_ALAT_not_strict_safe')
        safe, reason = _pose_safe(row, ctx, road, shape, budget, audit, cache, 'source')
        if not safe:
            raise PoseTimeRefusal('source_not_strict_safe_' + reason)
    if not ctx._in_goal(float(rows[-1, 1]), float(rows[-1, 2])):
        raise PoseTimeRefusal('source_has_no_first_safe_goal_entry')
    if any(ctx._in_goal(float(r[1]), float(r[2])) for r in rows[:-1]):
        raise PoseTimeRefusal('source_rows_continue_after_first_goal')
    audit['source_full_action_math_verified'] = True
    audit['source_dual_footprint_GT_TTC_lateral_ALAT_verified'] = True
    audit['source_all_native_states_including_initial_and_first_goal_checked'] = True
    audit['source_bad_pose_frames'] = 0


def _source_nodes(rows, frames, index, prefix):
    anchor = tuple(map(float, rows[index]))
    previous = tuple(map(float, rows[index-1])) if index else (
        tuple(map(float, prefix[-1])) if len(prefix) else None)
    omega = None if previous is None else _angle(anchor[3]-previous[3])/(anchor[0]-previous[0])
    nodes = [_Node(anchor, float(frames[index]['previous_acceleration_mps2']),
                   float(frames[index]['final_steering_rad']), previous, omega,
                   -1, (), (), 0, True)]
    for end in range(index+EDGE_STEPS, len(rows)+EDGE_STEPS-1, EDGE_STEPS):
        end = min(end, len(rows)-1)
        begin = index+nodes[-1].chain_steps
        if end <= begin:
            break
        edge_actions = tuple((float(frames[i]['t']), float(frames[i]['command_mps']),
                              float(frames[i]['final_steering_rad'])) for i in range(begin, end))
        row, previous = tuple(map(float, rows[end])), tuple(map(float, rows[end-1]))
        nodes.append(_Node(row, float(frames[end-1]['actual_acceleration_mps2']),
            float(frames[end-1]['final_steering_rad']), previous,
            _angle(row[3]-previous[3])/float(row[0]-previous[0]), len(nodes)-1,
            tuple(tuple(map(float, r)) for r in rows[begin+1:end+1]), edge_actions,
            end-index, True,
            tuple(float(frames[i]['actual_acceleration_mps2']) for i in range(begin, end))))
        if end == len(rows)-1:
            break
    return nodes


def _reachable_steering(row, new_speed, previous_omega, target, length, rot):
    speed = float(row[4])
    delta = max(-STEERING_LIMIT, min(STEERING_LIMIT, float(target)))
    if speed > 1e-9:
        # 原P先确定v_next，交当前中心帧和新终点one-sided可达侧加区间。
        bound = ALAT_LIMIT/max(speed, float(new_speed), 1e-9)
        low, high = -bound, bound
        if previous_omega is not None:
            low = max(low, -2.*ALAT_LIMIT/speed-previous_omega)
            high = min(high, 2.*ALAT_LIMIT/speed-previous_omega)
        if low > high:
            raise PoseTimeRefusal('empty_hard_lateral_reachable_interval')
        omega = speed/length*1.7*math.tan(delta)
        delta = math.atan(max(low, min(high, omega))*length/(1.7*speed))
    if rot is not None and speed > .1:
        bound = math.atan(rot*length/(1.7*speed*speed))
        delta = max(-bound, min(bound, delta))
    return delta


def _advance_edge(node, target_delta, command, ctx, shape, rot, budget, audit,
                  road, cache, first_action):
    """一个图边；尚未硬可行的中间帧绝不入图或作完整终态缓存。"""
    row, previous = tuple(node.row), node.previous_row
    acceleration, previous_omega = node.last_a, node.previous_omega
    rows, actions, accelerations = [], [], []
    for step in range(EDGE_STEPS):
        budget.check('edge_step', search=True)
        if audit['scalar_steps'] >= MAX_SCALAR_STEPS:
            raise _SearchStop('scalar_step_cap')
        if node.chain_steps+step >= MAX_CHAIN_STEPS:
            return None, 'chain_step_cap'
        audit['scalar_steps'] += 1
        t, x, y, yaw, speed = row
        selected_command = float(first_action[1]) if node.chain_steps == 0 and step == 0 else command
        new_speed, acceleration = track_step(speed, acceleration, selected_command, float(ctx.dt))
        if node.chain_steps == 0 and step == 0:
            delta = float(first_action[2])
        else:
            try:
                delta = _reachable_steering(row, new_speed, previous_omega,
                                           target_delta, shape['length'], rot)
            except PoseTimeRefusal as error:
                return None, str(error)
        if not math.isfinite(delta) or abs(delta) > STEERING_LIMIT+1e-12:
            return None, 'steering_domain'
        # first_action也是原rot后finalδ；原链的真实结果仍必须兑现。
        if rot is not None and speed > .1:
            bound = math.atan(rot*shape['length']/(1.7*speed*speed))
            if abs(delta) > bound+1e-12:
                return None, 'original_rot_first_action_mismatch'
        following = (t+ctx.dt, x+speed*np.cos(yaw)*ctx.dt, y+speed*np.sin(yaw)*ctx.dt,
                     yaw+speed/shape['length']*1.7*np.tan(delta)*ctx.dt, new_speed)
        if following[0] > float(ctx.max_t)+1e-9:
            return None, 'scene_horizon'
        good, _, _ = _closed_alat(previous, row, following)
        if not good:
            return None, 'central_or_recorder_ALAT'
        safe, reason = _pose_safe(following, ctx, road, shape, budget, audit, cache, 'edge')
        if not safe:
            return None, reason
        actions.append((t, selected_command, delta))
        accelerations.append(float(acceleration))
        rows.append(tuple(map(float, following)))
        previous_omega = _angle(following[3]-row[3])/float(ctx.dt)
        previous, row = row, following
        if ctx._in_goal(float(row[1]), float(row[2])):
            terminal_safe, _, _ = _terminal_alat(previous, row)
            if not terminal_safe:
                return None, 'terminal_one_sided_or_recorder_ALAT'
            break
    return _Node(tuple(row), float(acceleration), float(actions[-1][2]), tuple(previous),
                 previous_omega, -1, tuple(rows), tuple(actions),
                 node.chain_steps+len(actions), False, tuple(accelerations)), 'edge_hard_feasible'


def _primitives(node, goal_center, length):
    bearing = math.atan2(goal_center[1]-node.row[2], goal_center[0]-node.row[1])
    sign = math.copysign(1., _angle(bearing-node.row[3]))
    deltas = [sign*STEERING_LIMIT, node.delta, 0., -sign*STEERING_LIMIT]
    corner = math.sqrt(CORNER_COMMAND_ALAT*float(length)/(1.7*math.tan(STEERING_LIMIT)))
    commands = [.1, max(.1, min(12., corner)), max(.1, min(12., float(node.row[4]))), 12.]
    result = []
    for delta in deltas:
        for command in commands:
            value = (float(delta), float(command))
            if value not in result:
                result.append(value)
    return result


def _priority(node, vertices, start_time):
    xy = np.asarray(node.row[1:3])
    distance = np.maximum(0., np.maximum(vertices.min(axis=0)-xy, xy-vertices.max(axis=0)))
    # 固定weighted best-first，目的为有界终态发现；不是admissible最优证明。
    return (float(node.row[0]-start_time)
            +PRIORITY_DISTANCE_WEIGHT*float(np.linalg.norm(distance))/12.)


def _extract(nodes, index):
    parts, actions = [], []
    cursor = index
    while nodes[cursor].parent >= 0:
        node = nodes[cursor]
        parts.append(node.edge_rows)
        actions.append(node.edge_actions)
        cursor = node.parent
    rows = np.asarray([nodes[cursor].row]+[r for part in reversed(parts) for r in part])
    controls = np.asarray([a for part in reversed(actions) for a in part])
    return rows, controls


def _chain_proof(nodes, index):
    """只导出所选单链的原parent/history/实际a，不新增任何传播或查询。"""
    chain = []
    cursor = index
    while cursor >= 0:
        node = nodes[cursor]
        chain.append(dict(node_id=cursor, parent_id=node.parent, row=list(node.row),
            last_acceleration_mps2=node.last_a, final_steering_rad=node.delta,
            previous_row=None if node.previous_row is None else list(node.previous_row),
            previous_omega_radps=node.previous_omega, source_chain_node=node.source_node,
            chain_steps=node.chain_steps, edge_rows=[list(r) for r in node.edge_rows],
            edge_actions=[list(a) for a in node.edge_actions],
            edge_actual_acceleration_mps2=list(node.edge_accelerations)))
        cursor = node.parent
    return list(reversed(chain))


def _search(nodes, ctx, shape, rot, road, budget, audit, cache, vertices, first_action):
    start_time = float(nodes[0].row[0])
    source_terminal = len(nodes)-1
    incumbent_time, incumbent = float(nodes[source_terminal].row[0]), None
    heap, seen = [], set()
    for index, node in enumerate(nodes[:-1]):
        key = _node_key(node, start_time, float(ctx.dt))
        seen.add(key)
        heapq.heappush(heap, (_priority(node, vertices, start_time), index))
    goal_center = vertices.mean(axis=0)
    try:
        while heap:
            budget.check('graph_expansion', search=True)
            if audit['expanded'] >= MAX_EXPANDED:
                raise _SearchStop('expanded_node_cap')
            _, index = heapq.heappop(heap)
            node = nodes[index]
            if node.row[0] >= incumbent_time-1e-8 or node.chain_steps >= MAX_CHAIN_STEPS:
                continue
            audit['expanded'] += 1
            for delta, command in _primitives(node, goal_center, shape['length']):
                budget.check('edge_proposal', search=True)
                if audit['edge_proposals'] >= MAX_EDGE_PROPOSALS:
                    raise _SearchStop('edge_proposal_cap')
                audit['edge_proposals'] += 1
                child, reason = _advance_edge(node, delta, command, ctx, shape, rot,
                                              budget, audit, road, cache, first_action)
                if child is None:
                    refusals = audit['refused_edges']
                    refusals[reason] = refusals.get(reason, 0)+1
                    continue
                child = _Node(child.row, child.last_a, child.delta, child.previous_row,
                    child.previous_omega, index, child.edge_rows, child.edge_actions,
                    child.chain_steps, False, child.edge_accelerations)
                if child.row[0] >= incumbent_time-1e-8:
                    continue
                key = _node_key(child, start_time, float(ctx.dt))
                if key in seen:
                    audit['approx_pose_prunes'] += 1
                    continue
                seen.add(key)
                nodes.append(child)
                child_index = len(nodes)-1
                audit['accepted_edges'] += 1
                if ctx._in_goal(float(child.row[1]), float(child.row[2])):
                    incumbent_time, incumbent = float(child.row[0]), child_index
                    audit['faster_hard_feasible_terminals'] += 1
                else:
                    heapq.heappush(heap, (_priority(child, vertices, start_time), child_index))
    except _SearchStop as stop:
        audit['search_stop_reason'] = str(stop)
    audit.update(node_count=len(nodes), retained_key_count=len(seen),
                 source_incumbent_terminal_s=float(nodes[source_terminal].row[0]),
                 faster_incumbent_terminal_s=None if incumbent is None else incumbent_time)
    if incumbent is None:
        return None
    budget.check('before_single_chain_proof')
    audit['final_chain_parent_history_proof'] = _chain_proof(nodes, incumbent)
    audit['model_actual_acceleration_mps2'] = [a for node in audit['final_chain_parent_history_proof']
                                              for a in node['edge_actual_acceleration_mps2']]
    budget.check('after_single_chain_proof')
    return _extract(nodes, incumbent)


def _final_gate(ctx, modeled, prefix, datasets_dir, budget, audit):
    """F16独立最终门；每个新API前后核总8s，不调用旧F15终门。"""
    budget.check('before_final_API_import')
    from score_result import lateral_acceleration, road_boundary_collision, traffic_metrics
    all_rows = np.vstack([prefix, modeled])
    shape = {key: float(ctx.ego_shape[value]) for key, value in
        [('width', 'width'), ('length', 'length'), ('head', 'locationPoint2Head'), ('rear', 'locationPoint2Rear')]}
    for row in all_rows:
        t, x, y, yaw, _ = map(float, row)
        budget.check('final_runtime_lookup')
        audit['runtime_queries'] += 1
        if ctx._road_hit(x, y, yaw):
            raise PoseTimeRefusal('final_model_runtime_bitmap_footprint_collision')
        budget.check('final_GT_t_plus_dt')
        background = _background_at_time(ctx, t, budget, audit, 'final')
        budget.check('final_runtime_actor_collision')
        audit['runtime_actor_collision_queries'] += 1
        if ctx._collide(x, y, yaw, background, str(np.around(t, 3))):
            raise PoseTimeRefusal('final_model_gt_t_plus_dt_vehicle_collision')
        budget.check('final_raw_traffic')
        audit['traffic_pose_checks'] += 1
        safe, reason = _traffic_at_pose(row, background, shape)
        if not safe:
            raise PoseTimeRefusal('final_raw_pose_' + reason)
        budget.check('after_final_raw_pose')
    budget.check('before_final_recorder_frame')
    frame = _recorder_frame(ctx, all_rows)
    budget.check('before_final_official_road')
    audit['final_official_road_queries'] += 1
    collision, check = road_boundary_collision(frame, Path(f'{ctx.name}_result.csv'), Path(datasets_dir))
    budget.check('before_final_official_traffic')
    audit['final_official_traffic_queries'] += 1
    traffic = traffic_metrics(frame, 3., .8)
    budget.check('before_final_recorder_ALAT')
    alat = lateral_acceleration(frame)
    budget.check('after_final_recorder_ALAT')
    audit['final_model_gate'] = dict(road_check=check, road_collision=bool(collision),
        vehicle_collision=bool(traffic['vehicle_collision']),
        ttc_violation_frames=int(np.count_nonzero(traffic['ttc_violation'])),
        lateral_distance_violation_frames=int(np.count_nonzero(traffic['lateral_violation'])),
        recorder_alat_violation_frames=int(np.count_nonzero(alat > .4)),
        recorder_max_alat_mps2=float(np.max(alat)), minimum_ttc_s=traffic['minimum_ttc_s'],
        minimum_lateral_distance_m=traffic['minimum_lateral_distance_m'],
        traffic_phase='absolute_t_plus_dt', all_native_prefix_initial_first_goal_included=True,
        no_official_total_score_computed=True, independent_F16_gate=True)
    budget.check('after_final_audit')
    if (collision or check != 'bitmap_footprint' or traffic['vehicle_collision']
            or np.any(traffic['ttc_violation']) or np.any(traffic['lateral_violation'])
            or np.any(alat > .4)):
        raise PoseTimeRefusal('final_model_official_bitmap_traffic_or_recorder_alat_refused')
    budget.check('before_final_first_goal_check')
    if not ctx._in_goal(float(modeled[-1, 1]), float(modeled[-1, 2])):
        raise PoseTimeRefusal('final_model_misses_official_goal')
    if any(ctx._in_goal(float(r[1]), float(r[2])) for r in modeled[:-1]):
        raise PoseTimeRefusal('final_model_continues_after_official_first_goal')
    budget.check('after_final_qualification')


def build_candidate(base, online_rows, action_frames, ctx, ctrl_at_anchor, datasets_dir,
                    budget_s=STATIC_BUDGET_S):
    started = time.perf_counter()
    audit = dict(mechanism='safe_source_seeded_bounded_kinodynamic_pose_time_graph',
        source='complete_strict_safe_native_same_object_online_actions', external_csv_used=False,
        parameter_grid=False, graph_searches=0, source_seeds=1, optimizer_runs=0,
        complete_dynamic_rollouts=0, env_calls=0, score_calls=0, policy_count=0, accepted=False,
        static_budget_s=float(budget_s), final_reserve_s=FINAL_RESERVE_S,
        hard_wall_cap_claimed=False, expanded=0, edge_proposals=0, scalar_steps=0,
        source_action_math_steps_reserved=0,
        scalar_step_counter_semantics='reserved_source_receipt_steps_plus_actual_graph_steps',
        accepted_edges=0, faster_hard_feasible_terminals=0, approx_pose_prunes=0,
        source_pose_checks=0, edge_pose_checks=0, runtime_queries=0,
        actual_polygon_queries=0, GT_queries=0, traffic_pose_checks=0,
        GT_synth_calls=0, GT_memo_hits=0,
        runtime_actor_collision_queries=0, final_official_road_queries=0,
        final_official_traffic_queries=0,
        pose_cache_hits=0, map_patch_loads=0, final_static_proposal_count=0, refused_edges={},
        max_expanded=MAX_EXPANDED, max_edge_proposals=MAX_EDGE_PROPOSALS,
        max_scalar_steps=MAX_SCALAR_STEPS, edge_steps=EDGE_STEPS,
        max_chain_steps=MAX_CHAIN_STEPS, reused_component_role='math_road_recorder_token_no_F15_model_optimizer_builder')
    audit.update(search_order='fixed_weighted_best_first',
        priority_distance_weight=PRIORITY_DISTANCE_WEIGHT,
        heuristic_admissibility_or_optimality_claimed=False,
        corner_command_ALAT_mps2=CORNER_COMMAND_ALAT,
        steering_target_count_at_most=4, speed_command_count_at_most=4,
        primitives_per_node_at_most=16)
    try:
        if not math.isfinite(float(budget_s)) or not FINAL_RESERVE_S < float(budget_s) <= STATIC_BUDGET_S:
            raise PoseTimeRefusal('static_budget_must_be_above_reserve_and_at_most_eight_seconds')
        budget = _Budget(budget_s, started)
        if not getattr(base, 'tracking', False) or not callable(getattr(base, 'steering', None)):
            raise PoseTimeRefusal('source_not_tracking_with_steering')
        rows, frames, steering = _rows_and_frames(online_rows, action_frames)
        if np.any(np.abs(steering) > STEERING_LIMIT+1e-12):
            raise PoseTimeRefusal('source_steering_exceeds_original_point_six_domain')
        dt = float(ctx.dt)
        if abs(dt-.1) > 1e-12 or np.max(np.abs(np.diff(rows[:, 0])-dt)) > 1e-8:
            raise PoseTimeRefusal('original_point_one_second_chain_required')
        if np.max(np.abs(rows[0]-np.asarray(ctx.prefix[-1]))) > 1e-7:
            raise PoseTimeRefusal('online_rows_must_begin_at_current_prefix_state')
        vertices = _vertices(np.column_stack([ctx.goal['x'], ctx.goal['y']]))
        index, anchor = preturn_anchor(rows, frames, vertices)
        audit['anchor'] = anchor
        if index is None:
            raise PoseTimeRefusal(anchor['reason'])
        if not 0 <= index < len(frames):
            raise PoseTimeRefusal('invalid_single_source_anchor')
        actual = np.asarray([getattr(ctrl_at_anchor, k) for k in
                             ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v')], dtype=float)
        previous = float(ctrl_at_anchor._last_acc)
        if (not np.isfinite(actual).all() or np.max(np.abs(actual-rows[index, 1:])) > 1e-8
                or not math.isfinite(previous) or abs(previous-frames[index]['previous_acceleration_mps2']) > 1e-12):
            raise PoseTimeRefusal('true_before_process_anchor_controller_required')
        shape = {key: float(ctx.ego_shape[value]) for key, value in
            [('width', 'width'), ('length', 'length'), ('head', 'locationPoint2Head'), ('rear', 'locationPoint2Rear')]}
        if not all(math.isfinite(v) and v > 0. for v in shape.values()):
            raise PoseTimeRefusal('positive_actual_truck_shape_required')
        rot_map = getattr(ctrl_at_anchor, '_rot_scenes', {})
        override = rot_map.get(ctx.name)
        if override is None:
            override = rot_map.get('_default')
        rot = None if override in (None, 'off', {}) else float(override.get('a_rot', .38))
        if rot is not None and not .1 <= rot <= .4:
            raise PoseTimeRefusal('bounded_original_rot_chain_required')
        external_prefix = np.asarray(ctx.prefix[:-1], dtype=float).reshape(-1, 5)
        prefix = np.vstack([external_prefix, rows[:index]])
        all_rows = np.vstack([external_prefix, rows])
        audit['max_query_cache_entries'] = len(all_rows)+MAX_SCALAR_STEPS
        road = _prepare_road(ctx, all_rows, shape, datasets_dir, budget, audit)
        cache = {}
        _verify_source(rows, frames, external_prefix, ctx, road, shape, budget, audit, cache)
        budget.check('before_graph', search=True)
        if len(rows)-index-1 > MAX_CHAIN_STEPS:
            raise PoseTimeRefusal('source_seed_suffix_exceeds_chain_step_cap')
        nodes = _source_nodes(rows, frames, index, external_prefix)
        audit.update(graph_searches=1, source_chain_nodes=len(nodes),
            original_suffix_time_s=float(rows[-1, 0]-rows[index, 0]),
            prefix_switch_time_s=float(rows[index, 0]), first_suffix_command_mps=float(frames[index]['command_mps']),
            first_suffix_steering_rad=float(steering[index]), original_rot_acceleration_mps2=rot,
            initial_acceleration_mps2=previous, pose_only_pruning_is_exact_dominance=False,
            first_action_fixed_to_source=True, goal_yaw_is_free=True)
        audit.update(nonterminal_endpoint_central_ALAT_pending_until_next_edge=True,
            terminal_one_sided_ALAT_required=True,
            safety_scope='official_native_point_one_second_whole_pose_states',
            continuous_swept_safety_claimed=False)
        first_action = (rows[index, 0], frames[index]['command_mps'], steering[index])
        proposal = _search(nodes, ctx, shape, rot, road, budget, audit, cache, vertices, first_action)
        if proposal is None:
            raise PoseTimeRefusal('no_faster_complete_hard_feasible_pose_time_corridor')
        modeled, actions = proposal
        if (len(actions) > MAX_CHAIN_STEPS or len(actions) == 0
                or modeled[-1, 0] >= rows[-1, 0]-1e-8
                or np.max(np.abs(modeled[0]-rows[index])) > 1e-8
                or np.max(np.abs(actions[0]-np.asarray(first_action))) > 1e-10):
            raise PoseTimeRefusal('final_chain_start_horizon_or_improvement_mismatch')
        audit.update(final_static_proposal_count=1, model_steps=len(actions),
            model_terminal_s=float(modeled[-1, 0]), model_suffix_time_s=float(modeled[-1, 0]-modeled[0, 0]),
            model_rows=modeled.tolist(), model_actions=actions.tolist(),
            max_executed_abs_steering_rad=float(np.max(np.abs(actions[:, 2]))),
            only_hard_feasible_complete_terminal_retained=True)
        budget.check('before_final_qualification')
        _final_gate(ctx, modeled, prefix, datasets_dir, budget, audit)
        budget.check('before_single_candidate')
        candidate = PoseTimeCorridorCand(base, actions, dt, modeled)
        budget.check('after_single_candidate')
        audit.update(candidate=candidate.label,
            reason='one_direct_action_pose_time_corridor_requires_complete_same_object_execution',
            prefix_commands_and_steering_delegated_to_original_base=True,
            original_controller_and_jerk_unchanged=True, elapsed_s=time.perf_counter()-started)
        budget.check('after_candidate_bookkeeping')
        elapsed = time.perf_counter()-started
        if elapsed >= float(budget_s):
            raise PoseTimeRefusal('static_budget_before_candidate_return')
        audit.update(accepted=True, policy_count=1, elapsed_s=elapsed)
        budget.check('before_candidate_return')
        return candidate, audit
    except _SearchStop as error:
        audit.update(accepted=False, policy_count=0,
                     reason='no_static_budget_for_search_and_final_qualification',
                     search_stop_reason=str(error), elapsed_s=time.perf_counter()-started)
        return None, audit
    except Exception as error:
        audit.update(accepted=False, policy_count=0,
                     reason=str(error) if isinstance(error, FeasiblePortalRefusal) else 'proposal_exception',
                     error=dict(type=type(error).__name__, message=str(error)), elapsed_s=time.perf_counter()-started)
        return None, audit
