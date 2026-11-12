"""F15：在线整车姿态走廊门户与主弯/可弯出口的硬可达单提案。

单位为米、秒、弧度；rows 为 (N,5) 的绝对 t/x/y/yaw/v，actions 为
(N-1,3) 的 t/速度命令/最终转向。安全source只是见证和单seed，不以
任意goal边点作靶，不强制零转向出口。只有全部硬约束可行的终态才保留。
本模块不执行完整政策预测或Env；最终静态门仍不是收益证明。
"""

import math
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize


MAX_EVALUATIONS = 360
MAX_MODEL_STEPS = 430
STATIC_BUDGET_S = 8.
MAX_PATCH_PIXELS = 2500000
SUSTAINED_FRAMES = 10
PRETURN_LEAD_S = 1.
ALAT_LIMIT = .391
BODY_GRID_SPACING_M = .5
GOAL_MARGIN_M = 1e-8


class FeasiblePortalRefusal(ValueError):
    pass


class _Budget:
    def __init__(self, seconds, started=None):
        self.seconds = float(seconds)
        self.started = time.perf_counter() if started is None else started

    def check(self, stage):
        if time.perf_counter() - self.started >= self.seconds:
            raise FeasiblePortalRefusal('static_budget_' + stage)


def _angle(value):
    return (value + np.pi) % (2. * np.pi) - np.pi


def _rows_and_frames(rows, action_frames):
    rows = np.asarray(rows, dtype=float)
    if (rows.ndim != 2 or rows.shape[1] != 5 or len(rows) < 15
            or not np.isfinite(rows).all() or np.any(rows[:, 4] < 0.)
            or np.any(np.diff(rows[:, 0]) <= 0.)):
        raise FeasiblePortalRefusal('finite_increasing_online_five_state_rows_required')
    frames = list(action_frames)
    if len(frames) != len(rows) - 1:
        raise FeasiblePortalRefusal('one_complete_source_action_per_transition_required')
    steering = []
    for i, frame in enumerate(frames):
        command, delta, t = (float(frame[k]) for k in
                            ('command_mps', 'final_steering_rad', 't'))
        if (not all(math.isfinite(v) for v in (command, delta, t))
                or not .1 <= command <= 12. or abs(delta) > 1.
                or abs(t - rows[i, 0]) > 1e-9):
            raise FeasiblePortalRefusal('source_action_time_or_domain_mismatch')
        steering.append(delta)
    return rows, frames, np.asarray(steering)


def _vertices(vertices):
    vertices = np.asarray(vertices, dtype=float)
    if (vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 3
            or not np.isfinite(vertices).all()):
        raise FeasiblePortalRefusal('finite_goal_polygon_required')
    if np.linalg.norm(vertices[0] - vertices[-1]) < 1e-8:
        vertices = vertices[:-1]
    edges = np.roll(vertices, -1, axis=0) - vertices
    lengths = np.linalg.norm(edges, axis=1)
    area = np.sum(vertices[:, 0] * np.roll(vertices[:, 1], -1)
                  - vertices[:, 1] * np.roll(vertices[:, 0], -1))
    if len(vertices) < 3 or np.any(lengths <= 1e-8) or abs(area) <= 1e-8:
        raise FeasiblePortalRefusal('nondegenerate_goal_polygon_required')
    direction = math.copysign(1., area)
    turns = edges[:, 0] * np.roll(edges[:, 1], -1) - edges[:, 1] * np.roll(edges[:, 0], -1)
    if np.any(direction * turns < -1e-8):
        raise FeasiblePortalRefusal('convex_goal_portal_required')
    return vertices


def preturn_anchor(rows, action_frames, vertices):
    """单次同向旧域饱和前的未饱和事件；不扫场名、锚点或时间。"""
    rows, _, steering = _rows_and_frames(rows, action_frames)
    vertices = _vertices(vertices)
    yaw = np.unwrap(rows[:, 3])
    for onset in range(len(steering) - SUSTAINED_FRAMES + 1):
        segment = steering[onset:onset + SUSTAINED_FRAMES]
        if (np.any(np.abs(segment) < .6 - 1e-8)
                or not np.all(np.sign(segment) == np.sign(segment[0]))):
            continue
        finish = onset + SUSTAINED_FRAMES
        while (finish < len(steering) and abs(steering[finish]) >= .6 - 1e-8
               and np.sign(steering[finish]) == np.sign(segment[0])):
            finish += 1
        sweep = float(yaw[finish] - yaw[onset])
        if abs(sweep) < .7:
            continue
        eligible = np.flatnonzero((rows[:onset, 0] <= rows[onset, 0] - PRETURN_LEAD_S + 1e-8)
                                  & (np.abs(steering[:onset]) < .55))
        if not len(eligible):
            return None, dict(reason='sustained_turn_has_no_nonsaturated_lead_anchor',
                              onset_index=onset)
        index = int(eligible[-1])
        bearing = math.atan2(*(vertices.mean(axis=0) - rows[index, 1:3])[::-1])
        if np.sign(_angle(bearing - rows[index, 3])) != np.sign(segment[0]):
            return None, dict(reason='first_sustained_turn_not_goal_facing', onset_index=onset)
        if finish >= len(rows) - 3:
            return None, dict(reason='no_post_turn_source_corridor', onset_index=onset)
        return index, dict(reason='first_sustained_same_direction_source_turn', index=index,
            cut_time_s=float(rows[index, 0]), onset_index=onset,
            saturation_end_index=finish, saturation_frames=finish-onset,
            saturation_onset_s=float(rows[onset, 0]), saturation_end_s=float(rows[finish, 0]),
            heading_change_rad=sweep, turn_sign=int(np.sign(segment[0])),
            nonsaturated_anchor_steering_rad=float(steering[index]),
            preturn_lead_s=float(rows[onset, 0]-rows[index, 0]))
    return None, dict(reason='no_sustained_significant_old_domain_saturation')


def track_step(speed, previous_acceleration, command, dt):
    requested = max(-6., min(2.5, 2. * (command-speed)))
    acceleration = max(previous_acceleration-1., min(previous_acceleration+1., requested))
    return max(0., speed+acceleration*dt), acceleration


def _source_contract(rows, frames, dt, length):
    """只核验既有在线动作收据，不重推进一个新source政策。"""
    for i, frame in enumerate(frames):
        previous = float(frame['previous_acceleration_mps2'])
        actual = float(frame['actual_acceleration_mps2'])
        if (frame.get('pass_frame') is not True or not math.isfinite(previous)
                or not math.isfinite(actual) or not -6. <= previous <= 2.5
                or not -6. <= actual <= 2.5
                or (i and abs(previous-float(frames[i-1]['actual_acceleration_mps2'])) > 1e-12)):
            raise FeasiblePortalRefusal('complete_valid_original_source_execution_required')
        speed, acceleration = track_step(rows[i, 4], previous, frame['command_mps'], dt)
        expected = np.array([rows[i, 0]+dt,
            rows[i, 1]+rows[i, 4]*np.cos(rows[i, 3])*dt,
            rows[i, 2]+rows[i, 4]*np.sin(rows[i, 3])*dt,
            rows[i, 3]+rows[i, 4]/length*1.7*np.tan(frame['final_steering_rad'])*dt, speed])
        if abs(acceleration-actual) > 1e-12 or np.max(np.abs(expected-rows[i+1])) > 1e-8:
            raise FeasiblePortalRefusal('source_same_code_old_speed_or_P_chain_mismatch')


class LocalFeasiblePortalTurnCand:
    kind = 'F15_LOCAL_FEASIBLE_PORTAL_TURN'
    tracking = True
    _local_exit_policy = True

    def __setattr__(self, key, value):
        if getattr(self, '_sealed', False):
            raise AttributeError('feasible_portal_candidate_is_immutable')
        object.__setattr__(self, key, value)

    def __init__(self, base, actions, dt, model_rows):
        if not getattr(base, 'tracking', False) or not callable(getattr(base, 'steering', None)):
            raise FeasiblePortalRefusal('tracking_base_with_steering_required')
        actions = np.array(actions, dtype=float, copy=True)
        rows = np.array(model_rows, dtype=float, copy=True)
        if (not math.isfinite(float(dt)) or abs(float(dt)-.1) > 1e-12
                or actions.ndim != 2 or actions.shape[1] != 3
                or not 1 <= len(actions) <= MAX_MODEL_STEPS
                or rows.shape != (len(actions)+1, 5)
                or not np.isfinite(actions).all() or not np.isfinite(rows).all()
                or np.any(rows[:, 4] < 0.)
                or np.max(np.abs(np.diff(rows[:, 0])-dt)) > 1e-9
                or np.max(np.abs(actions[:, 0]-rows[:-1, 0])) > 1e-10
                or np.any((actions[:, 1] < .1) | (actions[:, 1] > 12.))
                or np.any(np.abs(actions[:, 2]) > 1.)):
            raise FeasiblePortalRefusal('invalid_final_immutable_action_table')
        self.base, self.dt = base, float(dt)
        self.actions, self.model_rows = actions, rows
        self.cut_time, self.end_time = float(actions[0, 0]), float(rows[-1, 0])
        self.label = 'LOCAL_FEASIBLE_PORTAL_TURN_I1_' + base.label
        actions.setflags(write=False)
        rows.setflags(write=False)
        self._sealed = True

    def command(self, ctrl, bank_v, t, budget=None):
        t = float(t)
        packet = getattr(ctrl, '_local_exit_frame', None)
        if (packet is not None and packet[0] is self and packet[1] == id(ctrl)
                and (not packet[5] or abs(packet[2]-t) < 1e-8)):
            raise RuntimeError('feasible_portal_duplicate_or_unconsumed_frame')
        pose = tuple(float(getattr(ctrl, k)) for k in ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v'))
        if not all(math.isfinite(v) for v in (*pose, t)):
            raise ValueError('nonfinite_feasible_portal_pose_or_time')
        suffix = t+1e-7 >= self.cut_time
        ctrl._local_exit_frame = (self, id(ctrl), t, suffix, pose, False, False)
        if suffix:
            index = int(round((t-self.cut_time)/self.dt))
            if not 0 <= index < len(self.actions):
                raise RuntimeError('feasible_portal_model_horizon_exhausted')
            if abs(t-self.actions[index, 0]) > 1e-8:
                raise RuntimeError('feasible_portal_command_not_on_frozen_action_frame')
            command = float(self.actions[index, 1])
        else:
            # 原F12也用同一token名；保存其pending帧，转向时透明恢复。
            saved = getattr(ctrl, '_feasible_portal_base_frame', None)
            ctrl._local_exit_frame = (saved[3] if isinstance(saved, tuple)
                                      and saved[0] is self and saved[1] == id(ctrl) else None)
            try:
                command = float(self.base(ctrl, bank_v, t))
                base_packet = getattr(ctrl, '_local_exit_frame', None)
                ctrl._feasible_portal_base_frame = (self, id(ctrl), t, base_packet)
            finally:
                ctrl._local_exit_frame = (self, id(ctrl), t, suffix, pose, False, False)
        if not math.isfinite(command) or not .1 <= command <= 12.:
            raise ValueError('invalid_feasible_portal_command')
        ctrl._local_exit_frame = (self, id(ctrl), t, suffix, pose, False, True)
        return command

    def __call__(self, ctrl, bank_v, t):
        return self.command(ctrl, bank_v, t)

    def steering(self, ctrl):
        packet = getattr(ctrl, '_local_exit_frame', None)
        pose = tuple(float(getattr(ctrl, k)) for k in ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v'))
        if (packet is None or packet[0] is not self or packet[1] != id(ctrl)
                or packet[5] or not packet[6] or packet[4] != pose):
            raise RuntimeError('feasible_portal_steering_requires_same_unique_command')
        if packet[3]:
            index = int(round((packet[2]-self.cut_time)/self.dt))
            delta = float(self.actions[index, 2])
        else:
            saved = getattr(ctrl, '_feasible_portal_base_frame', None)
            if (not isinstance(saved, tuple) or saved[0] is not self
                    or saved[1] != id(ctrl) or saved[2] != packet[2]):
                raise RuntimeError('feasible_portal_prefix_base_command_packet_missing')
            ctrl._local_exit_frame = saved[3]
            try:
                delta = float(self.base.steering(ctrl))
                ctrl._feasible_portal_base_frame = (self, id(ctrl), packet[2],
                                                    getattr(ctrl, '_local_exit_frame', None))
            finally:
                ctrl._local_exit_frame = packet
        if not math.isfinite(delta) or abs(delta) > 1.:
            raise ValueError('invalid_feasible_portal_steering')
        ctrl._local_exit_frame = packet[:5] + (True, True)
        return delta


class _RoadClearance:
    """有界局部黑像素距离场；矩形内部格点球覆盖，绝非只查四角。"""
    def __init__(self, image, extent, shape):
        from scipy.ndimage import distance_transform_edt
        self.extent = tuple(map(float, extent))
        self.image = np.asarray(image)
        if (self.image.ndim != 2 or not self.image.size
                or self.image.size > MAX_PATCH_PIXELS):
            raise FeasiblePortalRefusal('bounded_local_map_patch_required')
        self.res_x = (extent[1]-extent[0])/self.image.shape[1]
        self.res_y = (extent[3]-extent[2])/self.image.shape[0]
        if not 0. < self.res_x <= .2 or not 0. < self.res_y <= .2:
            raise FeasiblePortalRefusal('bounded_bitmap_resolution_required')
        drivable = self.image >= 128
        self.field = distance_transform_edt(np.pad(drivable, 1, constant_values=False),
                                           sampling=(self.res_y, self.res_x))[1:-1, 1:-1]
        self.quantization_margin = 2.*math.hypot(self.res_x, self.res_y)
        self.shape = dict(shape)
        rear, head, width = (float(shape[k]) for k in ('rear', 'head', 'width'))
        longitudinal = np.linspace(-rear, head, int(math.ceil((head+rear)/BODY_GRID_SPACING_M))+1)
        lateral = np.linspace(-width/2., width/2., int(math.ceil(width/BODY_GRID_SPACING_M))+1)
        self.longitudinal, self.lateral = (a.ravel() for a in np.meshgrid(longitudinal, lateral))
        self.cover_radius = .5*math.hypot(longitudinal[1]-longitudinal[0], lateral[1]-lateral[0])

    def minimum(self, poses):
        poses = np.asarray(poses, dtype=float)
        cosine, sine = np.cos(poses[:, 3, None]), np.sin(poses[:, 3, None])
        xs = poses[:, 1, None]+cosine*self.longitudinal-sine*self.lateral
        ys = poses[:, 2, None]+sine*self.longitudinal+cosine*self.lateral
        ix = np.floor((xs-self.extent[0])/self.res_x).astype(int)
        iy = np.floor((ys-self.extent[2])/self.res_y).astype(int)
        if (np.any(ix < 0) or np.any(iy < 0) or np.any(ix >= self.image.shape[1])
                or np.any(iy >= self.image.shape[0])):
            return -1.
        return float(np.min(self.field[iy, ix])-self.cover_radius-self.quantization_margin)

    def witness_margin(self, pose):
        """与官方同构的完整车身PIL polygon；含内部黑点、头尾不对称。"""
        from PIL import Image, ImageDraw
        _, x, y, yaw, _ = map(float, pose)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        shape = self.shape
        local = np.array([[shape['head'], shape['width']/2.],
                          [shape['head'], -shape['width']/2.],
                          [-shape['rear'], -shape['width']/2.],
                          [-shape['rear'], shape['width']/2.]])
        px = (x+local[:, 0]*cosine-local[:, 1]*sine-self.extent[0])/self.res_x
        py = (y+local[:, 0]*sine+local[:, 1]*cosine-self.extent[2])/self.res_y
        if (np.any(px < 0) or np.any(py < 0) or np.any(px >= self.image.shape[1])
                or np.any(py >= self.image.shape[0])):
            raise FeasiblePortalRefusal('source_witness_outside_local_map_patch')
        left, bottom = int(math.floor(px.min())), int(math.floor(py.min()))
        right = min(self.image.shape[1], int(math.ceil(px.max()))+1)
        top = min(self.image.shape[0], int(math.ceil(py.max()))+1)
        footprint = Image.new('1', (right-left, top-bottom), 0)
        ImageDraw.Draw(footprint).polygon(list(zip(px-left, py-bottom)), fill=1)
        mask = np.asarray(footprint, dtype=bool)
        values = self.field[bottom:top, left:right][mask]
        if not len(values) or np.min(values) <= 0.:
            raise FeasiblePortalRefusal('source_witness_actual_truck_polygon_collision')
        return float(np.min(values)-self.quantization_margin)


class PoseCorridor:
    """source完整车身白像素见证的连通姿态球，q=(station,lateral,yaw)。"""
    def __init__(self, rows, margins, body_radius):
        self.rows = np.array(rows, dtype=float, copy=True)
        self.margins = np.array(margins, dtype=float, copy=True)
        self.body_radius = float(body_radius)
        if (self.rows.ndim != 2 or self.rows.shape[1] != 5 or len(self.rows) < 3
                or self.margins.shape != (len(self.rows),)
                or not np.isfinite(self.rows).all() or not np.isfinite(self.margins).all()
                or not math.isfinite(self.body_radius) or self.body_radius <= 0.
                or np.any(self.margins <= 0.)):
            raise FeasiblePortalRefusal('positive_actual_truck_pose_witness_margins_required')
        segments = np.linalg.norm(np.diff(self.rows[:, 1:3], axis=0), axis=1)
        if np.any(segments < 1e-6):
            raise FeasiblePortalRefusal('regular_forward_source_corridor_required')
        metric = segments+self.body_radius*np.abs(_angle(np.diff(self.rows[:, 3])))
        if np.any(metric > self.margins[:-1]+self.margins[1:]):
            raise FeasiblePortalRefusal('source_pose_witness_corridor_not_connected')
        self.station = np.r_[0., np.cumsum(segments)]
        for value in (self.rows, self.margins, self.station):
            value.setflags(write=False)

    def margin(self, rows):
        rows = np.asarray(rows, dtype=float)
        distance = np.linalg.norm(rows[:, None, 1:3]-self.rows[None, :, 1:3], axis=2)
        metric = distance+self.body_radius*np.abs(_angle(rows[:, None, 3]-self.rows[None, :, 3]))
        slack = self.margins[None, :]-metric
        return slack.max(axis=1), slack.argmax(axis=1)

    def project(self, rows):
        rows = np.asarray(rows, dtype=float)
        starts = self.rows[:-1, 1:3]
        edges = np.diff(self.rows[:, 1:3], axis=0)
        lengths = np.linalg.norm(edges, axis=1)
        relative = rows[:, None, 1:3]-starts[None, :, :]
        parameter = np.clip(np.sum(relative*edges[None, :, :], axis=2)/lengths**2, 0., 1.)
        residual = relative-parameter[:, :, None]*edges[None, :, :]
        near = np.argmin(np.sum(residual**2, axis=2), axis=1)
        index = np.arange(len(rows))
        fraction = parameter[index, near]
        station = self.station[near]+fraction*lengths[near]
        lateral = (edges[near, 0]*residual[index, near, 1]
                   - edges[near, 1]*residual[index, near, 0])/lengths[near]
        heading = self.rows[near, 3]+fraction*_angle(self.rows[near+1, 3]-self.rows[near, 3])
        return station, lateral, _angle(rows[:, 3]-heading)

    def portal_audit(self, row):
        value = np.asarray(row, dtype=float).reshape(1, 5)
        margins, witness = self.margin(value)
        station, lateral, yaw = self.project(value)
        return dict(station_m=float(station[0]), lateral_m=float(lateral[0]),
                    yaw_offset_rad=float(yaw[0]), witness_index=int(witness[0]),
                    actual_truck_pose_slack_m=float(margins[0]),
                    pose_t_x_y_yaw_v=value[0].tolist())


def _build_corridor(ctx, rows, finish, shape, datasets_dir, budget, audit):
    import score_embed
    import score_result
    budget.check('before_local_clearance')
    radius = math.hypot(max(shape['head'], shape['rear']), shape['width']/2.)
    padding = 2.*radius+2.
    bounds = (float(rows[:, 1].min()-padding), float(rows[:, 2].min()-padding),
              float(rows[:, 1].max()+padding), float(rows[:, 2].max()+padding))
    score_embed.install_map_cache()
    patch = score_result.load_map_patch(Path(datasets_dir), Path(f'{ctx.name}_result.csv'), bounds)
    audit['map_patch_loads'] += 1
    if patch is None:
        raise FeasiblePortalRefusal('official_bitmap_local_patch_unavailable')
    road = _RoadClearance(*patch, shape)
    budget.check('after_local_clearance')
    exit_rows = rows[finish:]
    margins = []
    for row in exit_rows:
        budget.check('source_pose_witnesses')
        rounded = np.asarray([round(float(row[0]), 1)]
                             + [round(float(v), 4) for v in row[1:]])
        perturbation = np.linalg.norm(rounded[1:3]-row[1:3])+radius*abs(_angle(rounded[3]-row[3]))
        margin = min(road.witness_margin(row), road.witness_margin(rounded)-perturbation)
        audit['source_full_polygon_witness_checks'] += 2
        margins.append(margin)
    corridor = PoseCorridor(exit_rows, margins, radius)
    if not ctx._in_goal(float(exit_rows[-1, 1]), float(exit_rows[-1, 2])):
        raise FeasiblePortalRefusal('source_corridor_has_no_safe_goal_entry')
    audit['corridor'] = dict(source_start_index=int(finish), witness_count=len(exit_rows),
        source_station_length_m=float(corridor.station[-1]),
        minimum_full_polygon_clearance_m=float(np.min(margins)),
        maximum_full_polygon_clearance_m=float(np.max(margins)),
        body_radius_m=radius, actual_shape=shape, local_patch_bounds=list(bounds),
        local_patch_extent=list(road.extent), local_patch_pixels=int(road.image.size),
        body_cover_point_count=len(road.longitudinal),
        body_cover_radius_m=road.cover_radius, pixel_quantization_margin_m=road.quantization_margin,
        connected_source_to_goal=True, source_yaw_is_seed_not_terminal_equality=True,
        pose_metric='translation_norm + actual_corner_radius*abs(wrapped_yaw_difference)',
        source_seed_terminal_portal=corridor.portal_audit(exit_rows[-1]))
    return road, corridor


def _model(start, previous_acceleration, dt, length, knot_times,
           steering_nodes, command_nodes, rot_acceleration):
    steps = int(math.ceil(float(knot_times[-1])/dt-1e-10))
    if not 2 <= steps <= MAX_MODEL_STEPS:
        raise FeasiblePortalRefusal('local_action_model_step_cap')
    t, x, y, yaw, speed = map(float, start)
    acceleration = float(previous_acceleration)
    rows, actions = [[t, x, y, yaw, speed]], []
    for _ in range(steps):
        elapsed = t-float(start[0])
        command = float(np.interp(elapsed, knot_times, command_nodes))
        delta = float(np.interp(elapsed, knot_times, steering_nodes))
        if rot_acceleration is not None and speed > .1:
            bound = math.atan(rot_acceleration*length/(1.7*speed*speed))
            if abs(delta) > bound:
                delta = math.copysign(bound, delta)
        actions.append((t, command, delta))
        new_speed, acceleration = track_step(speed, acceleration, command, dt)
        x, y = x+speed*np.cos(yaw)*dt, y+speed*np.sin(yaw)*dt
        yaw += speed/length*1.7*np.tan(delta)*dt
        speed, t = new_speed, t+dt
        rows.append((t, x, y, yaw, speed))
    return np.asarray(rows), np.asarray(actions)


def _goal_margin(rows, vertices):
    edges = np.roll(vertices, -1, axis=0)-vertices
    direction = math.copysign(1., np.sum(vertices[:, 0]*np.roll(vertices[:, 1], -1)
                                        - vertices[:, 1]*np.roll(vertices[:, 0], -1)))
    relative = rows[:, None, 1:3]-vertices[None, :, :]
    signed = direction*(edges[None, :, 0]*relative[:, :, 1]
                        - edges[None, :, 1]*relative[:, :, 0])/np.linalg.norm(edges, axis=1)
    return signed.min(axis=1)


def _hard_constraints(modeled, actions, exit_index, prefix, road, corridor, vertices,
                      max_duration):
    goal = _goal_margin(modeled, vertices)
    hits = np.flatnonzero(goal > GOAL_MARGIN_M)
    if len(hits):
        terminal = int(hits[0])
        modeled, actions = modeled[:terminal+1], actions[:terminal]
    else:
        terminal = len(modeled)-1
    all_rows = np.vstack([prefix, modeled])
    alat = np.abs(all_rows[:, 4]*np.gradient(np.unwrap(all_rows[:, 3]), all_rows[:, 0]))
    local_alat = alat[max(0, len(prefix)-1):]
    exit_valid = 0 <= exit_index < len(modeled)-1
    if exit_valid:
        tail = modeled[exit_index:]
        margin, _ = corridor.margin(tail)
        station, _, _ = corridor.project(tail)
        corridor_min = float(margin.min())
        forward = float(np.min(np.diff(station))) if len(station) > 1 else -1.
    else:
        corridor_min, forward = -1., -1.
    constraints = np.array([max_duration-(modeled[-1, 0]-modeled[0, 0]),
        ALAT_LIMIT-float(local_alat.max()), road.minimum(modeled), corridor_min,
        forward+1e-8, float(goal.max())-GOAL_MARGIN_M,
        float(terminal-exit_index-1)])
    return constraints, modeled, actions, dict(
        constraint_names=['horizon', 'original_P_jerk_central_difference_ALAT',
                          'whole_body_cover_road_clearance', 'connected_exit_pose_corridor',
                          'exit_forward_station', 'goal_center_entry', 'goal_after_exit_portal'],
        constraint_slacks=constraints.tolist(), hard_feasible=bool(np.all(constraints >= 0.)),
        proxy_max_alat_mps2=float(local_alat.max()),
        exit_index=int(exit_index), model_steps=len(actions))


def _recorder_frame(ctx, all_rows):
    import score_embed
    rows = np.asarray(all_rows)
    times = np.asarray([round(float(t), 1) for t in rows[:, 0]])
    ego = {key: np.asarray([round(float(v), 4) for v in rows[:, column]])
           for key, column in [('x', 1), ('y', 2), ('yaw', 3), ('v', 4)]}
    background = {}
    for actor, table in ctx.traj.items():
        if actor == 'ego':
            continue
        values = np.full((len(rows), 4), np.nan)
        for i, t in enumerate(times):
            item = table.get(str(np.around(t, 3)))
            if item is not None:
                values[i] = [round(float(item[k]), 4) for k in ('x', 'y', 'yaw_rad', 'v_mps')]
        shape = table['shape']
        background[str(actor)] = dict(x=values[:, 0], y=values[:, 1], yaw=values[:, 2],
            v=values[:, 3], width=float(shape['width']), head=float(shape['locationPoint2Head']),
            rear=float(shape['locationPoint2Rear']))
    shape = ctx.ego_shape
    return score_embed.build_frame(times, ego, background, terminal=4,
        ego_shape=dict(width=shape['width'], length=shape['length'],
                       head=shape['locationPoint2Head'], rear=shape['locationPoint2Rear']))


def _final_gate(ctx, modeled, prefix, datasets_dir, budget, audit):
    """一个最终硬可行终态；独立runtime/GT/真实polygon/recorder门。"""
    for row in modeled[1:]:
        budget.check('final_runtime_GT')
        t, x, y, yaw, _ = map(float, row)
        background, _ = ctx._synth(t)
        if t > .5 and ctx._collide(x, y, yaw, background, str(np.around(t, 3))):
            raise FeasiblePortalRefusal('final_model_gt_t_plus_dt_vehicle_collision')
        if ctx._road_hit(x, y, yaw):
            raise FeasiblePortalRefusal('final_model_runtime_bitmap_footprint_collision')
    budget.check('before_official_final_model_gate')
    from score_result import lateral_acceleration, road_boundary_collision, traffic_metrics
    frame = _recorder_frame(ctx, np.vstack([prefix, modeled]))
    collision, check = road_boundary_collision(frame, Path(f'{ctx.name}_result.csv'), Path(datasets_dir))
    traffic = traffic_metrics(frame, 3., .8)
    alat = lateral_acceleration(frame)
    audit['final_model_gate'] = dict(road_check=check, road_collision=bool(collision),
        vehicle_collision=bool(traffic['vehicle_collision']),
        ttc_violation_frames=int(np.count_nonzero(traffic['ttc_violation'])),
        lateral_distance_violation_frames=int(np.count_nonzero(traffic['lateral_violation'])),
        recorder_alat_violation_frames=int(np.count_nonzero(alat > .4)),
        recorder_max_alat_mps2=float(np.max(alat)), minimum_ttc_s=traffic['minimum_ttc_s'],
        minimum_lateral_distance_m=traffic['minimum_lateral_distance_m'],
        traffic_phase='absolute_t_plus_dt', no_official_total_score_computed=True)
    budget.check('after_official_final_model_gate')
    if (collision or check != 'bitmap_footprint' or traffic['vehicle_collision']
            or np.any(traffic['ttc_violation']) or np.any(traffic['lateral_violation'])
            or np.any(alat > .4)):
        raise FeasiblePortalRefusal('final_model_official_bitmap_traffic_or_recorder_alat_refused')
    if not ctx._in_goal(float(modeled[-1, 1]), float(modeled[-1, 2])):
        raise FeasiblePortalRefusal('final_model_misses_official_goal')
    if any(ctx._in_goal(float(r[1]), float(r[2])) for r in modeled[:-1]):
        raise FeasiblePortalRefusal('convex_portal_and_official_first_goal_time_mismatch')


def build_candidate(base, online_rows, action_frames, ctx, ctrl_at_anchor, datasets_dir,
                    budget_s=STATIC_BUDGET_S):
    """一个source seed、一个硬约束optimizer、仅一个最终政策。"""
    started = time.perf_counter()
    audit = dict(mechanism='source_full_truck_pose_corridor_portal_joint_reachability',
        source='complete_strict_safe_same_object_online_execution', external_csv_used=False,
        parameter_grid=False, optimizer_runs=0, optimizer_seeds=1, evaluations=0,
        policy_count=0, accepted=False, complete_dynamic_rollouts=0, env_calls=0,
        score_calls=0, static_budget_s=float(budget_s), hard_wall_cap_claimed=False,
        budget_checked_between_evaluations=True, safety_certificate_claimed=False,
        only_hard_feasible_best_retained=True, infeasible_model_states_retained=False,
        map_patch_loads=0, source_full_polygon_witness_checks=0,
        final_static_proposal_count=0, feasible_iterates=0, refused_hard_constraints={})
    try:
        if not math.isfinite(float(budget_s)) or not 0. < float(budget_s) <= STATIC_BUDGET_S:
            raise FeasiblePortalRefusal('static_budget_must_not_exceed_eight_seconds')
        budget = _Budget(budget_s, started)
        if not getattr(base, 'tracking', False) or not callable(getattr(base, 'steering', None)):
            raise FeasiblePortalRefusal('source_not_tracking_with_steering')
        rows, frames, steering = _rows_and_frames(online_rows, action_frames)
        dt = float(ctx.dt)
        if abs(dt-.1) > 1e-12 or np.max(np.abs(np.diff(rows[:, 0])-dt)) > 1e-8:
            raise FeasiblePortalRefusal('original_point_one_second_chain_required')
        if np.max(np.abs(rows[0]-np.asarray(ctx.prefix[-1], dtype=float))) > 1e-7:
            raise FeasiblePortalRefusal('online_rows_must_begin_at_current_prefix_state')
        vertices = _vertices(np.column_stack([ctx.goal['x'], ctx.goal['y']]))
        index, anchor = preturn_anchor(rows, frames, vertices)
        audit['anchor'] = anchor
        if index is None:
            raise FeasiblePortalRefusal(anchor['reason'])
        shape = {key: float(ctx.ego_shape[value]) for key, value in
                 [('width', 'width'), ('length', 'length'),
                  ('head', 'locationPoint2Head'), ('rear', 'locationPoint2Rear')]}
        if not all(math.isfinite(v) and v > 0. for v in shape.values()):
            raise FeasiblePortalRefusal('positive_actual_truck_shape_required')
        _source_contract(rows, frames, dt, shape['length'])
        actual = np.array([getattr(ctrl_at_anchor, k) for k in
                           ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v')], dtype=float)
        previous = float(ctrl_at_anchor._last_acc)
        if (not np.isfinite(actual).all() or np.max(np.abs(actual-rows[index, 1:])) > 1e-8
                or not math.isfinite(previous) or not -6. <= previous <= 2.5
                or abs(previous-float(frames[index]['previous_acceleration_mps2'])) > 1e-12):
            raise FeasiblePortalRefusal('true_before_process_anchor_controller_required')
        rot = getattr(ctrl_at_anchor, '_rot_scenes', {})
        override = rot.get(ctx.name)
        if override is None:
            override = rot.get('_default')
        rot_acceleration = None if override in (None, 'off', {}) else float(override.get('a_rot', .38))
        if rot_acceleration is not None and not .1 <= rot_acceleration <= .4:
            raise FeasiblePortalRefusal('bounded_original_rot_chain_required')
        finish, onset = anchor['saturation_end_index'], anchor['onset_index']
        road, corridor = _build_corridor(ctx, rows[index:], finish-index, shape,
                                        datasets_dir, budget, audit)
        if 'corridor' in audit:
            audit['corridor']['source_online_start_index'] = int(finish)
        # 首个自然退弯完成帧；不是门户/节点网格，也不选择任意goal边。
        release_options = np.flatnonzero(np.abs(steering[finish:]) <= .1)
        if not len(release_options):
            raise FeasiblePortalRefusal('no_source_release_event_for_curved_exit')
        release = min(finish+int(release_options[0]), len(frames)-2)
        enter = max(.3, float(rows[onset, 0]-rows[index, 0]))
        hold = max(.3, float(rows[finish, 0]-rows[onset, 0]))
        release_duration = max(.3, float(rows[release, 0]-rows[finish, 0]))
        tail = max(1., float(rows[-1, 0]-rows[release, 0]))
        middle = (release+len(frames)-1)//2
        initial = np.array([float(np.max(np.abs(steering[onset:finish]))),
            float(np.median([f['command_mps'] for f in frames[onset:finish]])),
            float(frames[-1]['command_mps']), enter, hold, release_duration, tail,
            float(steering[release]), float(steering[middle])])
        max_duration = min(MAX_MODEL_STEPS*dt, float(ctx.max_t-rows[index, 0])-dt)
        bounds = [(.6, 1.), (.1, 3.), (.1, 12.), (.3, 3.), (.3, 18.),
                  (.3, 4.), (1., 18.), (-.6, .6), (-.6, .6)]
        if max_duration < 2.:
            raise FeasiblePortalRefusal('insufficient_local_action_horizon')
        initial = np.array([np.clip(x, lo, hi) for x, (lo, hi) in zip(initial, bounds)])
        prefix = np.vstack([np.asarray(ctx.prefix[:-1]).reshape(-1, 5), rows[:index]])
        best, last_key, last_values = None, None, None
        original_duration = float(rows[-1, 0]-rows[index, 0])

        def evaluate(variables):
            nonlocal best, last_key, last_values
            budget.check('optimizer_evaluation')
            key = np.asarray(variables, dtype=float).tobytes()
            if key == last_key:
                return last_values
            if audit['evaluations'] >= MAX_EVALUATIONS:
                raise FeasiblePortalRefusal('optimizer_evaluation_cap_reached')
            audit['evaluations'] += 1
            peak, turn_command, exit_command, entry, dwell, leave, duration, exit_delta, connection_delta = map(float, variables)
            times = np.array([0., entry, entry+dwell, entry+dwell+leave,
                              entry+dwell+leave+duration/2., entry+dwell+leave+duration])
            if times[-1] > max_duration:
                last_key, last_values = key, (float(times[-1]), np.full(7, -float(times[-1]-max_duration)))
                return last_values
            modeled, actions = _model(rows[index], previous, dt, shape['length'], times,
                [steering[index], anchor['turn_sign']*peak, anchor['turn_sign']*peak,
                 exit_delta, connection_delta, 0.],
                [float(frames[index]['command_mps']), turn_command, turn_command,
                 turn_command, exit_command, exit_command], rot_acceleration)
            exit_index = int(math.ceil(times[3]/dt-1e-10))
            constraints, modeled, actions, receipt = _hard_constraints(
                modeled, actions, exit_index, prefix, road, corridor, vertices, max_duration)
            budget.check('after_hard_constraints')
            duration = float(modeled[-1, 0]-modeled[0, 0])
            if receipt['hard_feasible']:
                audit['feasible_iterates'] += 1
                if best is None or duration < best['duration_s']:
                    best = dict(duration_s=duration, variables=list(map(float, variables)),
                                rows=modeled.copy(), actions=actions.copy(), receipt=receipt)
            else:
                for name, value in zip(receipt['constraint_names'], constraints):
                    if value < 0.:
                        counts = audit['refused_hard_constraints']
                        counts[name] = counts.get(name, 0)+1
            # Infeasible transient rows/actions are discarded; only scalar residuals memoized.
            last_key, last_values = key, (float(times[-1]), constraints)
            return last_values

        audit.update(optimizer_runs=1, parameter_order=['peak_abs_steering_rad',
            'turn_command_mps', 'exit_command_mps', 'entry_duration_s', 'hold_duration_s',
            'release_duration_s', 'curved_connector_duration_s', 'exit_steering_rad',
            'connector_steering_rad'], initial_variables=initial.tolist(), variable_bounds=bounds,
            optimizer_method='SLSQP_hard_inequalities', source_seed='current_online_actions_only',
            initial_acceleration_mps2=previous, original_rot_acceleration_mps2=rot_acceleration,
            original_suffix_time_s=original_duration, prefix_switch_time_s=float(rows[index, 0]),
            first_suffix_command_mps=float(frames[index]['command_mps']),
            first_suffix_steering_rad=float(steering[index]),
            terminal_goal_is_source_connected_pose_corridor=True,
            zero_steering_straight_tail_forced=False, portal_station_lateral_yaw_are_free=True)
        try:
            result = minimize(lambda x: evaluate(x)[0], initial, method='SLSQP', bounds=bounds,
                constraints=[dict(type='ineq', fun=lambda x: evaluate(x)[1])],
                options=dict(maxiter=30, ftol=1e-4, eps=.004))
            audit.update(optimizer_success=bool(result.success), optimizer_message=str(result.message))
        except FeasiblePortalRefusal as error:
            audit.update(optimizer_success=False, optimizer_message=str(error))
            # maxeval只停止优化，已有硬可行best仍须剩余预算内的全部最终门。
            # 墙钟耗尽立即拒绝，不在失败预算之后启动新查询。
            if str(error).startswith('static_budget_'):
                raise
            audit['optimizer_stopped_at_evaluation_cap'] = (
                str(error) == 'optimizer_evaluation_cap_reached')
        if best is None:
            raise FeasiblePortalRefusal('no_hard_feasible_source_corridor_portal_action_model')
        audit.update(variables=best['variables'], model_hard_constraints=best['receipt'],
                     model_suffix_time_s=best['duration_s'])
        if best['duration_s'] >= original_duration-1e-8:
            raise FeasiblePortalRefusal('no_faster_hard_feasible_corridor_proposal')
        modeled, actions = best['rows'], best['actions']
        if abs(actions[0, 2]-steering[index]) > 1e-10 or abs(actions[0, 1]-frames[index]['command_mps']) > 1e-10:
            raise FeasiblePortalRefusal('entry_action_continuity_failed')
        audit.update(final_static_proposal_count=1, model_steps=len(actions),
            model_terminal_s=float(modeled[-1, 0]), model_rows=modeled.tolist(),
            model_actions=actions.tolist(), max_executed_abs_steering_rad=float(np.max(np.abs(actions[:, 2]))),
            exit_portal=corridor.portal_audit(modeled[best['receipt']['exit_index']]),
            terminal_portal=corridor.portal_audit(modeled[-1]))
        budget.check('before_final_static_proposal')
        _final_gate(ctx, modeled, prefix, datasets_dir, budget, audit)
        budget.check('before_candidate_construction')
        candidate = LocalFeasiblePortalTurnCand(base, actions, dt, modeled)
        audit.update(accepted=True, policy_count=1, candidate=candidate.label,
            reason='one_hard_feasible_pose_corridor_proposal_requires_full_same_object_execution',
            original_controller_and_jerk_unchanged=True, terminal_yaw_is_free=True,
            prefix_commands_and_steering_delegated_to_original_base=True,
            elapsed_s=time.perf_counter()-started)
        return candidate, audit
    except Exception as error:
        audit.update(reason=str(error) if isinstance(error, FeasiblePortalRefusal) else 'proposal_exception',
                     error=dict(type=type(error).__name__, message=str(error)),
                     elapsed_s=time.perf_counter()-started)
        return None, audit
