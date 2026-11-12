"""局部新几何的实际曲率后缀重定时，不改变原执行前缀。

输入为本次完整同码安全预测的(t秒,x米,y米,yaw弧度,v米/秒)行。
只生成一个NEGFF后缀政策；空间表和负前馈不是安全或收益证书。
调用方仍须完整执行、前缀、最高先前安全分/HYST及同对象Env验收。
"""

import math
import sys

import numpy as np

from control_local_curvature_exit import LocalCurvatureExitCand


def refine_candidate(geometry, online_rows, ctx, iteration=1):
    """同一Trace后缀消费本次实际曲率，原base及控制器状态保持原对象。"""
    audit = dict(
        mechanism='local_geometry_executed_suffix_negative_feedforward',
        accepted=False, policy_count=0, complete_candidate_limit=1,
        parameter_grid=False, reference_uses_external_csv=False,
        row_source='complete_same_code_safe_geometry_prediction_exact_cut_suffix',
        suffix_curvature_source='actual_recorder_yaw_gradient_per_time_div_speed',
        acceleration_feedforward='negative_only', controller_p_gain=2.,
        feedforward_formula='selected_cap + min(0, .5*d(cap**2)/ds) / 2',
        prefix_controller_state_reset=False,
        initial_acceleration_source='unchanged_runtime_controller_last_acc',
        geometry_changed=False, geometric_profile_is_safety_certificate=False,
        complete_dynamic_rollouts=0, env_runs=0, real_gain_verified=False,
    )
    try:
        if type(iteration) is not int or iteration != 1:
            raise ValueError('only_one_suffix_refinement_iteration_allowed')
        if (not isinstance(geometry, LocalCurvatureExitCand)
                or geometry.kind != 'F11_LOCAL_CURVATURE_EXIT'):
            raise ValueError('original_local_geometry_policy_required')
        cut, dt, geometry_dt = float(geometry.cut_time), float(ctx.dt), float(geometry.dt)
        if (not math.isfinite(cut) or not math.isfinite(dt) or dt <= 0.
                or not math.isfinite(geometry_dt) or abs(geometry_dt - dt) > 1e-12):
            raise ValueError('finite_matching_cut_and_dt_required')
        if (getattr(geometry.suffix, 'kind', None) != 'F7'
                or not getattr(geometry.suffix, 'tracking', False)
                or not callable(getattr(geometry.suffix, 'steering', None))
                or hasattr(geometry.suffix, '_retime_s')):
            raise ValueError('unrefined_original_trace_suffix_required')
        rows = np.asarray(online_rows, dtype=float)
        if (rows.ndim != 2 or rows.shape[1] != 5 or len(rows) < 5
                or not np.isfinite(rows).all() or np.any(rows[:, 4] < 0.)):
            raise ValueError('finite_nonnegative_five_state_rows_required')
        if (np.any(np.diff(rows[:, 0]) <= 0.)
                or np.max(np.abs(np.diff(rows[:, 0]) - dt)) > 1e-7):
            raise ValueError('strictly_increasing_uniform_dt_required')
        initial = np.asarray(ctx.prefix[-1], dtype=float)
        if (initial.shape != (5,) or not np.isfinite(initial).all()
                or np.max(np.abs(rows[0] - initial)) > 1e-7):
            raise ValueError('complete_prediction_initial_state_required')
        cuts = np.flatnonzero(rows[:, 0] == cut)
        if len(cuts) != 1:
            raise ValueError('one_exact_absolute_cut_row_required')
        index = int(cuts[0])
        suffix_rows = np.array(rows[index:], dtype=float, copy=True)
        if len(suffix_rows) < 5:
            raise ValueError('at_least_five_suffix_rows_required')
        length = float(np.linalg.norm(np.diff(suffix_rows[:, 1:3], axis=0), axis=1).sum())
        if length < 1.:
            raise ValueError('suffix_distance_at_least_one_metre_required')
        delay = float(geometry.suffix.delay)
        if not math.isfinite(delay) or abs(delay - cut) > 1e-12:
            raise ValueError('original_trace_delay_must_equal_absolute_cut')
        backend = sys.modules.get('executed_retiming_ff')
        builder = getattr(backend, 'build_candidate', None)
        if backend is None or not callable(builder):
            raise ValueError('frozen_lab_negative_ff_module_must_already_be_loaded')
        retimed, profile = builder(geometry.suffix, suffix_rows, 1)
        if (retimed is None or getattr(retimed, 'base', None) is not geometry.suffix
                or getattr(retimed, 'kind', None) != 'F9_EXECUTED_RETIMING_NEGATIVE_FF'
                or getattr(getattr(retimed, 'steering', None), '__self__', None) is not geometry.suffix):
            raise ValueError('negative_ff_must_delegate_same_trace_object')
        candidate = LocalCurvatureExitCand(geometry.base, retimed, cut, dt)
        candidate.kind = 'F12_LOCAL_GEOMETRY_EXECUTED_SUFFIX'
        candidate.label = 'LOCAL_GEOMETRY_EXEC_SUFFIX_NEGFF_I1_' + geometry.label
        audit.update(
            accepted=True, reason='one_suffix_proposal_requires_complete_execution',
            policy_count=1, iteration=iteration, candidate=candidate.label, source_geometry=geometry.label,
            input_rows=len(rows), suffix_rows=len(suffix_rows), cut_index=index,
            cut_time_s=cut, dt_s=dt, suffix_length_m=length,
            suffix_initial_five_state=suffix_rows[0].tolist(),
            suffix_time_is_absolute=True, cut_row_included=True,
            cut_gradient_boundary='first_suffix_row_uses_forward_difference',
            prefix_base_same_object=candidate.base is geometry.base,
            suffix_trace_same_object=candidate.suffix.base is geometry.suffix,
            steering_delegated=True, suffix_speed_policy_changed=True,
            negative_ff_backend=getattr(backend, '__file__', None),
            retiming_profile=profile,
        )
        return candidate, audit
    except Exception as error:
        audit.update(reason=str(error), error_type=type(error).__name__)
        return None, audit
