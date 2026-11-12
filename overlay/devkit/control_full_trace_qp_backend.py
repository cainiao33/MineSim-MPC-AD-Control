"""F20 静态全轨迹安全适配；复用原 GT/位图查询，不调用控制器或搜索。"""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from control_full_trace_qp_contract import FullTraceRefusal, FullTraceSource, FullTraceState, require
from control_native_bank_phase_backend import NativeBankBackendAdapter, metered_original_trajectory
from control_feasible_portal_turn import _recorder_frame
from probe_control_pose_time_corridor import complete_alat_check


class FullTraceQPBackendAdapter:
    """只暴露 selected/pose/final safety；原 process 证明由 probe 另计。"""

    def __init__(self, source, datasets_dir):
        require(isinstance(source, FullTraceSource), 'actual_full_trace_source_required')
        self.source = source
        frames = tuple(SimpleNamespace(before=FullTraceState(*map(float, before)),
            after=FullTraceState(*map(float, after)))
            for before, after in zip(source.rows6[:-1], source.rows6[1:]))
        # 该结构仅给冻结原地图 bounds 查询使用，不制造 process/action 证据。
        geometry = SimpleNamespace(ctx=source.ctx, path=source.path, frames=frames)
        self._geometry = NativeBankBackendAdapter(geometry, datasets_dir, None, source.original_process)
        self.datasets_dir = Path(datasets_dir)
        self._meter = None
        self.evidence = self._geometry.evidence
        self.evidence.update(mechanism='F20_full_trace_static_physical_gates',
            original_process_calls=0, original_process_verified_by_static_adapter=False,
            graph_searches=0, phase_events=0, official_total_score_calls=0)

    def backend(self):
        from control_full_trace_qp import FullTraceBackend
        return FullTraceBackend(selected=self.selected, pose=self.pose,
            final_gate=self.final_gate, final_charge_bound=self.final_charge_bound,
            original_process=self.source.original_process)

    def selected(self):
        source = self.source
        ctx = source.ctx
        plan = getattr(ctx, 'plan', None)
        require(isinstance(plan, (tuple, list)) and len(plan) == 3
            and plan[0] is source.policy and plan[1] == source.result.get('score')
            and plan[2] is source.result.get('recs') and getattr(ctx, 'path', None) is source.path,
            'source_not_same_actual_live_first_window_plan_and_path')
        require(ctx.ego_shape is source.ego_shape and float(ctx.dt) == .1
            and source.online_same_object and not source.external_csv_used
            and source.complete_actual_verified,
            'source_actual_shape_dt_or_capture_changed')
        return source.policy, source.result

    def pose(self, state, meter):
        require(isinstance(state, FullTraceState) and all(math.isfinite(v) for v in state.values()),
            'finite_full_trace_pose_required')
        if self._meter is None:
            self._meter = meter
        require(self._meter is meter, 'one_shared_static_meter_required')
        meter.charge('pose_gates')
        value = self._geometry.pose(self.source.ctx, state, meter)
        meter.check('after_full_trace_pose')
        return value

    def final_charge_bound(self, n_steps):
        """只覆盖 source 已缓存的时间格，逐查询仍实际收费。

        每行pose9、GT recorder每actor get1、缓存覆盖1、recorder组装1、
        road1、ALAT1及completeALAT1，traffic每actor(至少1)另收费；
        一wrapper与四official调用，再预留逐动作3及32固定metadata单位。map须已准备，
        不把 memo 查询视为免费。若时格缓存不足，不能提供此上界。
        """
        require(type(n_steps) is int and 1 <= n_steps <= len(self.source.commands),
            'bounded_final_step_count_required')
        ctx = self.source.ctx
        require(self._geometry.road is not None, 'source_road_admission_required_before_reserve')
        require(self._meter is not None, 'source_meter_required_before_reserve')
        require(isinstance(ctx._memo, dict) and isinstance(ctx.traj, dict),
            'original_memo_and_actor_tables_required')
        rows = self.source.rows6[:n_steps + 1]
        self._meter.charge('final_time_coverage_reads', len(rows))
        entries = [ctx._memo.get(str(np.around(float(row[0]), 3))) for row in rows]
        require(all(isinstance(entry, (tuple, list)) and len(entry) == 2
            and isinstance(entry[0], dict) and isinstance(entry[1], dict) for entry in entries),
            'source_time_lattice_memo_coverage_required_for_final_bound')
        actor_count = sum(actor != 'ego' for actor in ctx.traj)
        self._meter.charge('final_actor_count_reads', max(1, len(ctx.traj)))
        bound = ((n_steps + 1) * (14 + actor_count + max(1, actor_count))
            + n_steps * 3 + max(1, len(ctx.traj)) + 32)
        self.evidence['final_reserve_bound'] = dict(n_steps=n_steps,
            row_count=n_steps + 1, actor_count=actor_count, scalar_upper_bound=bound,
            all_source_times_memo_covered=True, map_already_prepared=True,
            memo_queries_remain_charged=True, original_process_proof=False)
        self._meter.check('after_final_reserve_bound')
        return bound

    def _original_rot(self, raw, speed):
        config = self.source.rot_config
        if config not in (None, 'off', {}) and speed > .1:
            require(isinstance(config, dict), 'original_rot_mapping_required')
            a_rot = float(config.get('a_rot', .38))
            require(math.isfinite(a_rot) and a_rot > 0., 'original_rot_limit_required')
            cap = math.atan(a_rot * self.source.vehicle_length / (1.7 * speed * speed))
            if abs(raw) > cap:
                raw = math.copysign(cap, raw)
        return raw

    def final_gate(self, rows6, raw_actions, final_delta, meter):
        """核验纯模型完整first-goal链；不给未运行process授予证明。"""
        import score_result
        rows = np.asarray(rows6)
        actions, delta = np.asarray(raw_actions), np.asarray(final_delta)
        meter.charge('final_input_checks')
        require(rows.ndim == 2 and rows.shape[1] == 6 and len(rows) >= 2
            and actions.shape == (len(rows) - 1, 3) and delta.shape == (len(rows) - 1,)
            and all(np.isfinite(value).all() for value in (rows, actions, delta))
            and np.all(rows[:, 4] >= 0.) and np.all(np.abs(np.diff(rows[:, 0]) - .1) <= 1e-9)
            and np.max(np.abs(actions[:, 0] - rows[:-1, 0])) <= 1e-10
            and np.all((.1 <= actions[:, 1]) & (actions[:, 1] <= 12.))
            and np.all(np.abs(actions[:, 2]) <= .6 + 1e-12)
            and np.all(np.abs(delta) <= .6 + 1e-12), 'finite_complete_final_action_model_required')
        require(np.max(np.abs(rows[0] - self.source.rows6[0])) <= 1e-10
            and actions[0, 1] == self.source.commands[0]
            and actions[0, 2] == self.source.raw_delta[0], 'original_first_state_and_action_required')
        expected_times = self.source.rows6[:len(rows), 0]
        require(len(rows) <= len(self.source.rows6)
            and np.max(np.abs(rows[:, 0] - expected_times)) <= 1e-9,
            'final_chain_must_use_admitted_source_time_lattice')
        self.final_charge_bound(len(rows) - 1)
        meter.charge('final_input_frame_checks', len(actions))
        for i, action in enumerate(actions):
            meter.charge('final_action_algebra_checks')
            t, x, y, yaw, speed, previous = map(float, rows[i])
            requested = max(-6., min(2.5, 2. * (float(action[1]) - speed)))
            acceleration = max(previous - 1., min(previous + 1., requested))
            meter.charge('final_original_rot_checks')
            realized = self._original_rot(float(action[2]), speed)
            require(abs(realized - float(delta[i])) <= 1e-10,
                'final_model_original_rot_mismatch')
            expected = (t + .1, x + speed * math.cos(yaw) * .1,
                y + speed * math.sin(yaw) * .1,
                yaw + speed / self.source.vehicle_length * 1.7 * math.tan(realized) * .1,
                max(0., speed + .1 * acceleration), acceleration)
            require(np.max(np.abs(rows[i + 1] - expected)) <= 1e-8,
                'final_model_old_speed_P2_jerk_six_state_mismatch')
            meter.check('after_final_action_algebra')
        first_goal = None
        for i, row in enumerate(rows):
            value = self.pose(FullTraceState(*map(float, row)), meter)
            require(all((value.runtime_road_safe, value.bitmap_polygon_safe,
                value.GT_collision_free, value.ttc_safe, value.lateral_distance_safe)),
                'final_full_trace_pose_not_safe')
            if value.in_goal and first_goal is None:
                first_goal = i
        ctx = self.source.ctx
        actor_count = self.evidence['final_reserve_bound']['actor_count']
        meter.charge('final_recorder_row_work', len(rows))
        with metered_original_trajectory(ctx, meter):
            frame = meter.call('final_recorder_frame_adapters', _recorder_frame, ctx, rows[:, :5])
        meter.charge('final_official_road_row_work', len(rows))
        collision, check = meter.call('final_official_road_queries', score_result.road_boundary_collision,
            frame, Path(f'{ctx.name}_result.csv'), self.datasets_dir)
        meter.charge('final_official_traffic_actor_row_work', len(rows) * max(1, actor_count))
        traffic = meter.call('final_official_traffic_queries', score_result.traffic_metrics, frame, 3., .8)
        meter.charge('final_official_ALAT_row_work', len(rows))
        alat = meter.call('final_official_ALAT_queries', score_result.lateral_acceleration, frame)
        meter.charge('final_complete_ALAT_row_work', len(rows))
        full_alat = meter.call('final_complete_ALAT_checks', complete_alat_check, rows[:, :5])
        require(first_goal == len(rows) - 1 and check == 'bitmap_footprint' and not collision
            and not traffic['vehicle_collision'] and not np.any(traffic['ttc_violation'])
            and not np.any(traffic['lateral_violation']) and not np.any(alat > .4)
            and full_alat['pass_alat'], 'final_official_full_trace_safety_refused')
        self.evidence['final_official_checks'] = dict(first_goal_index=first_goal,
            row_count=len(rows), road_check=check, road_collision=bool(collision),
            vehicle_collision=bool(traffic['vehicle_collision']),
            ttc_bad_frames=int(np.count_nonzero(traffic['ttc_violation'])),
            lateral_bad_frames=int(np.count_nonzero(traffic['lateral_violation'])),
            recorder_ALAT_bad_frames=int(np.count_nonzero(alat > .4)), complete_alat_check=full_alat,
            original_process_calls=0, no_official_total_score_computed=True)
        meter.check('after_full_trace_final_gate')
        return dict(complete_first_goal_chain=True, actual_bitmap_polygon_all_rows=True,
            absolute_GT_all_rows=True, raw_and_recorder_ALAT_all_rows=True,
            TTC_and_lateral_all_rows=True, static_only=True,
            original_process_actions_verified=False, complete_policy_predictions=0, Env_runs=0)
