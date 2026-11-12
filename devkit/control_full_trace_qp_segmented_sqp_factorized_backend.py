"""Recenter affine proposals on a complete safe model using current native GT."""

from dataclasses import dataclass

import numpy as np

from control_full_trace_qp_contract import require, frozen_array
from control_full_trace_qp_segmented_gt_backend import FullTraceSegmentedGTQPBackendAdapter
from control_gt_separation_planes import source_pair_planes, PROPOSAL_GAP_M, NEAR_DISTANCE_M
from control_pose_time_corridor import _rounded


@dataclass(frozen=True)
class FullTraceSQPBackend:
    selected: object
    pose: object
    final_gate: object
    final_charge_bound: object
    original_process: object
    source_body_margins: object
    source_traffic_planes: object
    current_body_margins: object
    current_traffic_planes: object


class FullTraceSegmentedSQPFactorizedBackendAdapter(FullTraceSegmentedGTQPBackendAdapter):
    def backend(self):
        return FullTraceSQPBackend(
            selected=self.selected, pose=self.pose, final_gate=self.final_gate,
            final_charge_bound=self.final_charge_bound,
            original_process=self.source.original_process,
            source_body_margins=self.source_body_margins,
            source_traffic_planes=self.source_traffic_planes,
            current_body_margins=self.current_body_margins,
            current_traffic_planes=self.current_traffic_planes)

    def current_body_margins(self, rows6, meter):
        require(self._meter is meter and self._geometry.road is not None,
                'admitted_source_before_recenter_geometry')
        meter.charge('source_body_corridor_reads')
        margins = []
        for row in rows6[:, :5]:
            values = [meter.call('source_body_clearance_map_queries',
                                self._geometry.road.witness_margin, pose)
                      for pose in (row, _rounded(row))]
            require(all(np.isfinite(v) and v > 0. for v in values),
                    'positive_recenter_body_clearance_required')
            margins.append(min(values))
        result = frozen_array(margins, (len(rows6),))
        self.evidence.setdefault('recenter_body_corridors', []).append(dict(
            poses=len(result), minimum_margin_m=float(np.min(result)),
            query_count=2 * len(result), from_complete_safe_model=True,
            external_csv_used=False, affine_proposal_only=True))
        return result

    def current_traffic_planes(self, rows6, meter):
        require(self._meter is meter, 'admitted_source_before_recenter_GT')
        result, pairs, indices = [], 0, []
        for index, row in enumerate(rows6[:, :5]):
            if index % 10 and index != len(rows6) - 1:
                continue
            indices.append(index)
            background, _ = self._geometry.background(self.source.ctx, float(row[0]), meter)
            for recorded in (False, True):
                meter.charge('source_GT_plane_pose_variants')
                pose = _rounded(row) if recorded else row
                for actor_id, values in background.items():
                    actor = (dict(values, **{k: round(float(values[k]), 4)
                             for k in ('x', 'y', 'yaw_rad', 'v_mps')})
                             if recorded else values)
                    planes = source_pair_planes(pose, actor, self._geometry.shape,
                                                index, actor_id, recorded, meter)
                    pairs += 1
                    if planes:
                        meter.charge('source_GT_plane_retention', len(planes))
                        result.extend(planes)
        self.evidence.setdefault('recenter_GT_separation_planes', []).append(dict(
            from_current_live_native_GT=True, from_complete_safe_model=True,
            external_CSV_used=False, pairs=pairs, faces=len(result),
            proposal_gap_m=PROPOSAL_GAP_M, near_distance_m=NEAR_DISTANCE_M,
            proposal_indices=indices, source_indices=len(rows6),
            proposal_stride_frames=10, all_actual_point_one_traffic_gates_preserved=True,
            affine_proposal_only=True))
        return tuple(result)
