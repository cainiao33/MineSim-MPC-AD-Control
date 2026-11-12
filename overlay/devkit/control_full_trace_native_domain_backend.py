"""F33独立原生转向域最终门；只把动作域±.6调整为Env的±1。"""
import math
from pathlib import Path
import numpy as np
from control_full_trace_qp_contract import FullTraceState, require
from control_full_trace_qp_signed_turn_backend import FullTraceSignedTurnBackendAdapter
from control_native_bank_phase_backend import metered_original_trajectory
from control_feasible_portal_turn import _recorder_frame
from probe_control_pose_time_corridor import complete_alat_check


class FullTraceNativeDomainBackendAdapter(FullTraceSignedTurnBackendAdapter):
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
            and np.all(np.abs(actions[:, 2]) <= 1. + 1e-12)
            and np.all(np.abs(delta) <= 1. + 1e-12), 'finite_complete_final_action_model_required')
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
