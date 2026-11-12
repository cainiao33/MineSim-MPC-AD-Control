"""F17 静态提案的原银行 process/GT/整车安全适配。

调用者安装透明审计挂点，本适配逐查询消费 helper 的预算。交通相位只作
速度提案见证；最终安全以全部 0.1 秒整车状态及官方 recorder 数学为准。
"""

import math
from collections.abc import Mapping
from contextlib import contextmanager
from pathlib import Path

import numpy as np

from control_native_bank_phase import (
    BankPhaseBackend, FrameInputs, NativeBankPhaseRefusal, PhaseEvent,
    PoseEvidence, ProcessTrace,
)
from control_feasible_portal_turn import (
    FeasiblePortalRefusal, _RoadClearance, _recorder_frame,
)
from control_pose_time_corridor import _corners, _rounded, _traffic_at_pose
from probe_control_pose_time_corridor import complete_alat_check


def state_row(state):
    return [state.t, state.x, state.y, state.yaw, state.speed]


def candidate_rows(candidate):
    return [state_row(candidate.frames[0].before)] + [state_row(f.after) for f in candidate.frames]


def packet_trace(packet, original_process, path):
    if not packet['pass_frame']:
        raise NativeBankPhaseRefusal('original_bank_process_observation_failed')
    return ProcessTrace(
        process_identity=original_process, path_identity=path,
        process_calls=1, bank_profile_stage_calls=len(packet['bank_profile']),
        pursuit_calls=len(packet['pursuit']), geometry_overrides=0,
        preview_failed=packet['preview_failed'],
        branch='native_idm' if packet['native_IDM_source'] else 'track_P2',
        final_steering=packet['final_steering_rad'])


class _MeteredActorTable(Mapping):
    """透明原 GT 表视图，逐时间帧 lookup 在预算内；静态 shape 不改。"""
    def __init__(self, table, meter):
        self.table, self.meter = table, meter

    def __getitem__(self, key):
        if key == 'shape':
            return self.table[key]
        return self.meter.call('GT_actor_table_queries', self.table.__getitem__, key)

    def __iter__(self):
        return iter(self.table)

    def __len__(self):
        return len(self.table)

    def get(self, key, default=None):
        if key == 'shape':
            return self.table.get(key, default)
        return self.meter.call('GT_actor_table_queries', self.table.get, key, default)


@contextmanager
def metered_original_trajectory(ctx, meter):
    original = ctx.traj
    ctx.traj = {actor: table if actor == 'ego' else _MeteredActorTable(table, meter)
                for actor, table in original.items()}
    try:
        yield
    finally:
        ctx.traj = original


class NativeBankBackendAdapter:
    def __init__(self, source, datasets_dir, audit, original_process):
        self.source, self.datasets_dir = source, Path(datasets_dir)
        self.audit, self.original_process = audit, original_process
        self.road = None
        self.last_trace = None
        self.evidence = dict(GT_queries=0, GT_memo_hits=0, GT_synth_calls=0,
                             raw_traffic_queries=0, rounded_traffic_queries=0,
                             phase_witnesses=[], map_patch_loads=0)
        ctx = source.ctx
        self.shape = {key: float(ctx.ego_shape[value]) for key, value in
                      [('width', 'width'), ('length', 'length'),
                       ('head', 'locationPoint2Head'), ('rear', 'locationPoint2Rear')]}

    def backend(self):
        return BankPhaseBackend(
            selected=self.selected, observe=self.observe, process=self.process,
            trace=self.trace, pose=self.pose, events=self.events,
            final_gate=self.final_gate, process_identity=self.original_process)

    def selected(self, ctx):
        if ctx is not self.source.ctx:
            raise NativeBankPhaseRefusal('backend_ctx_identity_changed')
        plan = getattr(ctx, 'plan', None)
        if (not isinstance(plan, (tuple, list)) or len(plan) != 3
                or plan[0] is not self.source.policy
                or plan[1] != self.source.result.get('score')
                or plan[2] is not self.source.result.get('recs')):
            raise NativeBankPhaseRefusal('source_not_actual_first_window_selected_result')
        return self.source.policy, self.source.result

    def background(self, ctx, t, meter):
        meter.charge('GT_queries')
        self.evidence['GT_queries'] += 1
        key = str(np.around(t, 3))
        entry = ctx._memo.get(key)
        if entry is None:
            with metered_original_trajectory(ctx, meter):
                entry = meter.call('GT_synth_calls', ctx._synth, t)
            ctx._memo[key] = entry
            self.evidence['GT_synth_calls'] += 1
        else:
            meter.note('GT_memo_hits')
            self.evidence['GT_memo_hits'] += 1
        meter.check('after_original_GT_query')
        return entry

    def observe(self, ctx, state, meter):
        actors, future = self.background(ctx, state.t, meter)
        vehicles = {'ego': dict(x=state.x, y=state.y, yaw_rad=state.yaw,
                               v_mps=state.speed, acc_mpss=state.last_acc,
                               shape=ctx.ego_shape)}
        vehicles.update(actors)
        observation = dict(vehicle_info=vehicles,
                           test_setting=dict(ctx._ts_tmpl, t=state.t, end=-1))
        return FrameInputs(vehicles, self.source.path, future, observation)

    def process(self, ctrl, vehicles, path, future, observation):
        candidate = getattr(ctrl, '_sas_cand', None)
        self.audit.begin(candidate, 'proposal_original_bank_process')
        try:
            result = self.audit.process(self.original_process, ctrl, vehicles,
                                        path, future, observation)
            execution, _ = self.audit.finish(1)
        except BaseException:
            self.audit.candidate, self.audit.active_frame = None, None
            raise
        self.last_trace = (ctrl, packet_trace(execution['frames'][0],
                                              self.original_process, path))
        return result

    def trace(self, ctrl):
        if self.last_trace is None or self.last_trace[0] is not ctrl:
            raise NativeBankPhaseRefusal('missing_same_controller_original_process_trace')
        return self.last_trace[1]

    def prepare_road(self, ctx, meter):
        if self.road is not None:
            return self.road
        import score_embed
        import score_result
        meter.call('map_cache_install_calls', score_embed.install_map_cache)
        xy = np.asarray([[f.before.x, f.before.y] for f in self.source.frames]
                        + [[self.source.frames[-1].after.x, self.source.frames[-1].after.y]]
                        + list(zip(ctx.goal['x'], ctx.goal['y'])), dtype=float)
        padding = 2. * math.hypot(max(self.shape['head'], self.shape['rear']),
                                  self.shape['width'] / 2.) + 2.
        bounds = (float(xy[:, 0].min() - padding), float(xy[:, 1].min() - padding),
                  float(xy[:, 0].max() + padding), float(xy[:, 1].max() + padding))
        patch = meter.call('map_patch_loads', score_result.load_map_patch,
                           self.datasets_dir, Path(f'{ctx.name}_result.csv'), bounds)
        self.evidence['map_patch_loads'] += 1
        if patch is None:
            raise NativeBankPhaseRefusal('official_local_bitmap_unavailable')
        self.road = meter.call('road_raster_constructions', _RoadClearance, *patch, self.shape)
        self.evidence['road_region'] = dict(bounds=list(bounds), extent=list(self.road.extent),
            pixels=int(self.road.image.size), source_tube_only=False,
            safety_scope='official_discrete_point_one_second_whole_body',
            continuous_swept_safety_claimed=False)
        return self.road

    def pose(self, ctx, state, meter):
        road = self.prepare_road(ctx, meter)
        row = state_row(state)
        runtime_safe = not meter.call('runtime_road_queries', ctx._road_hit,
                                       state.x, state.y, state.yaw)
        bitmap_safe = True
        for value in (row, _rounded(row)):
            try:
                meter.call('bitmap_polygon_queries', road.witness_margin, value)
            except FeasiblePortalRefusal:
                bitmap_safe = False
        background, _ = self.background(ctx, state.t, meter)
        collision_free = not meter.call('runtime_actor_collision_queries', ctx._collide,
            state.x, state.y, state.yaw, background, str(np.around(state.t, 3)))
        raw_safe, _ = meter.call('raw_traffic_queries', _traffic_at_pose,
                                 row, background, self.shape)
        self.evidence['raw_traffic_queries'] += 1
        rounded_background = {
            actor: dict(values, **{k: round(float(values[k]), 4)
                                  for k in ('x', 'y', 'yaw_rad', 'v_mps')})
            for actor, values in background.items()}
        rounded_safe, _ = meter.call('rounded_traffic_queries', _traffic_at_pose,
                                     _rounded(row), rounded_background, self.shape)
        self.evidence['rounded_traffic_queries'] += 1
        in_goal = bool(meter.call('goal_pose_queries', ctx._in_goal, state.x, state.y))
        dx = max(min(ctx.goal['x']) - state.x, 0., state.x - max(ctx.goal['x']))
        dy = max(min(ctx.goal['y']) - state.y, 0., state.y - max(ctx.goal['y']))
        return PoseEvidence(state.t, bool(runtime_safe), bool(bitmap_safe),
                            bool(collision_free), bool(raw_safe and rounded_safe),
                            bool(raw_safe and rounded_safe), in_goal, math.hypot(dx, dy))

    def events(self, ctx, source, meter):
        """Actual-GT footprint proximity windows along this native online route.

        The nearest native station and inflated .8m polygon distance are proposal
        witnesses. They neither certify an actor-free crossing nor fix ego yaw.
        """
        from shapely.geometry import Polygon
        self.prepare_road(ctx, meter)
        states = [f.before for f in source.frames] + [source.frames[-1].after]
        xy = np.asarray([[s.x, s.y] for s in states])
        ego_radius = math.hypot(max(self.shape['head'], self.shape['rear']),
                                self.shape['width'] / 2.)
        groups = {}
        for state in states:
            background, _ = self.background(ctx, state.t, meter)
            for actor, value in background.items():
                meter.charge('phase_actor_proposal_checks')
                actor_shape = value['shape']
                actor_radius = math.hypot(max(float(actor_shape['locationPoint2Head']),
                                               float(actor_shape['locationPoint2Rear'])),
                                          float(actor_shape['width']) / 2.)
                distances = np.sum((xy - [float(value['x']), float(value['y'])]) ** 2, axis=1)
                index = int(np.argmin(distances))
                meter.check('after_phase_nearest_station')
                if distances[index] > (ego_radius + actor_radius + .8) ** 2:
                    continue
                reference = states[index]
                meter.charge('phase_polygon_proposal_queries', 2)
                ego_polygon = Polygon(_corners(reference.x, reference.y, reference.yaw,
                    self.shape['width'], self.shape['head'], self.shape['rear'])).convex_hull
                actor_polygon = Polygon(_corners(float(value['x']), float(value['y']),
                    float(value['yaw_rad']), float(actor_shape['width']),
                    float(actor_shape['locationPoint2Head']),
                    float(actor_shape['locationPoint2Rear']))).convex_hull
                if ego_polygon.distance(actor_polygon) >= .8:
                    continue
                # Fixed native 0.5-second station bins only organize witnesses;
                # they do not merge search nodes or alter any control trajectory.
                key = (str(actor), index // 5)
                windows = groups.setdefault(key, [])
                if not windows or state.t - windows[-1][3] > .1000001:
                    windows.append([reference.x, reference.y, state.t, state.t])
                else:
                    windows[-1][3] = state.t
                meter.check('after_phase_footprint_witness')
        ordered = sorted(
            [((key[0], key[1], serial), window)
             for key, windows in groups.items() for serial, window in enumerate(windows)],
            key=lambda item: (item[1][2], item[0]))
        if len(ordered) > 430:
            raise NativeBankPhaseRefusal('native_GT_phase_witness_count_exceeds_cap')
        self.evidence['phase_witnesses'] = [dict(actor=key[0], native_station_bin=key[1],
            window=key[2],
            x=v[0], y=v[1], blocked_start=v[2], blocked_end=v[3]) for key, v in ordered]
        return tuple(PhaseEvent(*v, key[0], source.policy, source.result, True)
                     for key, v in ordered)

    def final_gate(self, ctx, candidate, meter):
        import score_result
        rows = candidate_rows(candidate)
        poses = [candidate.frames[0].before] + [f.after for f in candidate.frames]
        first_goal = None
        for index, state in enumerate(poses):
            evidence = meter.call('final_pose_adapters', self.pose, ctx, state, meter)
            if not all((evidence.runtime_road_safe, evidence.bitmap_polygon_safe,
                        evidence.GT_collision_free, evidence.ttc_safe,
                        evidence.lateral_distance_safe)):
                raise NativeBankPhaseRefusal('final_complete_native_bank_pose_not_safe')
            if evidence.in_goal and first_goal is None:
                first_goal = index
        with metered_original_trajectory(ctx, meter):
            frame = meter.call('final_recorder_frame_adapters', _recorder_frame,
                               ctx, np.asarray(rows, dtype=float))
        road_collision, road_check = meter.call('final_official_road_queries',
            score_result.road_boundary_collision, frame, Path(f'{ctx.name}_result.csv'),
            self.datasets_dir)
        traffic = meter.call('final_official_traffic_queries', score_result.traffic_metrics,
                              frame, 3., .8)
        alat = meter.call('final_official_ALAT_queries', score_result.lateral_acceleration, frame)
        complete_alat = meter.call('final_complete_ALAT_checks', complete_alat_check, rows)
        valid = (first_goal == len(poses) - 1 and road_check == 'bitmap_footprint'
                 and not road_collision and not traffic['vehicle_collision']
                 and not np.any(traffic['ttc_violation'])
                 and not np.any(traffic['lateral_violation'])
                 and not np.any(alat > .4) and complete_alat['pass_alat'])
        self.evidence['final_official_checks'] = dict(first_goal_index=first_goal,
            row_count=len(poses), road_check=road_check, road_collision=bool(road_collision),
            vehicle_collision=bool(traffic['vehicle_collision']),
            ttc_bad_frames=int(np.count_nonzero(traffic['ttc_violation'])),
            lateral_bad_frames=int(np.count_nonzero(traffic['lateral_violation'])),
            recorder_ALAT_bad_frames=int(np.count_nonzero(alat > .4)),
            complete_alat_check=complete_alat, no_official_total_score_computed=True)
        if not valid:
            raise NativeBankPhaseRefusal('final_official_native_bank_chain_not_safe')
        return dict(complete_first_goal_chain=True, original_bank_process_actions_verified=True,
            actual_bitmap_polygon_all_rows=True, absolute_GT_all_rows=True,
            raw_and_recorder_ALAT_all_rows=True, TTC_and_lateral_all_rows=True,
            static_only=True, complete_policy_predictions=0, Env_runs=0)
