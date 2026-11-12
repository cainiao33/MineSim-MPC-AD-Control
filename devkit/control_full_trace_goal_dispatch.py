"""按当前提前帧的真实目标几何选一个优化器；保留原F40可用情形。"""

import math
import time

import numpy as np

import control_native_bank_phase as bank
from control_full_trace_qp_segmented_sqp_factorized import build_candidate as parent_builder
from control_full_trace_sqp_goal_error_compensation import build_candidate as polygon_builder
from control_live_goal_polygon import goal_polygon, face_distances


class DispatchMeter:
    def __init__(self):
        self.scalar_steps = 0
        self.counts = {}

    def charge(self, kind, count=1):
        if type(count) is not int or count <= 0:
            raise ValueError('positive routing charge required')
        self.scalar_steps += count
        self.counts[kind] = self.counts.get(kind, 0) + count


class SharedStartClock:
    """Only the budget starts before routing; real Env clocks are untouched."""
    def __init__(self, clock, started):
        self.clock, self.started, self.first = clock, started, True

    def __call__(self):
        if self.first:
            self.first = False
            return self.started
        return self.clock()


def build_candidate(source, identity_receipt, priors, backend, *, clock=time.perf_counter):
    started = clock()
    meter = DispatchMeter()
    route = dict(selected_builder='F40', criterion='earlier_point_inside_AABB_but_outside_true_convex_goal',
                 source_is_current_identity_qualified=False, external_csv_used=False,
                 optimizer_calls=1, extra_native_predictions=0, extra_Env_runs=0)
    chosen = parent_builder
    try:
        if not identity_receipt.valid_for(source):
            raise ValueError('routing requires current complete identity receipt')
        route['source_is_current_identity_qualified'] = True
        meter.charge('dispatch_prior_safe_score_reads', max(1, len(priors)))
        safe = [float(r['score']) for r in priors if bank._physically_safe(r)]
        threshold = max(safe) + .15
        if not 80. < threshold < 100.:
            raise ValueError('no bounded earlier target')
        duration_limit = 20. * float(source.ctx._exp_t) / (threshold - 80.)
        # FullTraceSource already verifies its increasing 0.1s state chain.
        # Match the parent strict time inequality, excluding its terminal.
        lo, hi = 1, len(source.commands)
        while lo < hi:
            middle = (lo + hi) // 2
            meter.charge('dispatch_earlier_target_time_queries')
            elapsed = float(source.rows6[middle, 0] - source.rows6[0, 0])
            if elapsed < duration_limit:
                lo = middle + 1
            else:
                hi = middle
        earlier = lo - 1
        if earlier < 1:
            raise ValueError('no earlier positive frame')
        polygon = goal_polygon(source, meter)
        point = source.rows6[earlier, 1:3]
        lower, upper = polygon.vertices.min(axis=0), polygon.vertices.max(axis=0)
        meter.charge('dispatch_actual_AABB_and_polygon_predicate')
        in_box = bool(np.all(point >= lower) and np.all(point <= upper))
        distances = face_distances(polygon, point, meter)
        outside_polygon = bool(np.any(distances > 0.))
        route.update(fixed_earlier_goal_index=earlier, duration_limit_s=duration_limit,
                     highest_safe_prior_plus_HYST=threshold, earlier_xy=point.tolist(),
                     inside_AABB=in_box, outside_true_polygon=outside_polygon,
                     polygon_vertices=polygon.vertices.tolist(), polygon_face_distances=distances.tolist(),
                     parent_first_QP_objective_would_refuse=in_box)
        if in_box and outside_polygon:
            chosen = polygon_builder
            route['selected_builder'] = 'F45'
    except Exception as error:
        route['fallback_to_original_builder'] = dict(type=type(error).__name__, message=str(error))
    candidate, audit = chosen(source, identity_receipt, priors, backend,
                              clock=SharedStartClock(clock, started))
    original_steps = audit['scalar_steps']
    audit['scalar_steps'] = original_steps + meter.scalar_steps
    for kind, count in meter.counts.items():
        audit['counters'][kind] = audit['counters'].get(kind, 0) + count
    if 'source_admission_scalar' in audit:
        audit['source_admission_scalar'] += meter.scalar_steps
        audit['optimization_stage_scalar_start'] += meter.scalar_steps
    elapsed = clock() - started
    route.update(routing_scalar_steps=meter.scalar_steps, original_builder_scalar_steps=original_steps,
                 shared_total_static_clock=True, actual_Env_clock_unchanged=True)
    audit.update(goal_geometry_dispatch=route, elapsed_s=elapsed)
    source_ok = audit.get('source_admission_scalar', 0) <= 15000
    if not math.isfinite(elapsed) or not 0. <= elapsed < 8. or audit['scalar_steps'] > 864000 or not source_ok:
        candidate = None
        audit.update(accepted=False, candidate_count=0, reason='goal_dispatch_original_total_resource_limit')
    return candidate, audit
