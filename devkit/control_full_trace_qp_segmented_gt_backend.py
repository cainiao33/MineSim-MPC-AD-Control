"""F38原生源整车margin；每次实际位图查询在原静态计量中执行。"""

import numpy as np

from control_full_trace_qp_contract import require, frozen_array
from control_full_trace_qp_segmented_gt import FullTraceBackend
from control_full_trace_native_domain_backend import FullTraceNativeDomainBackendAdapter
from control_pose_time_corridor import _rounded


class FullTraceSegmentedGTQPBackendAdapter(FullTraceNativeDomainBackendAdapter):
    def backend(self):
        return FullTraceBackend(selected=self.selected, pose=self.pose,
            final_gate=self.final_gate, final_charge_bound=self.final_charge_bound,
            original_process=self.source.original_process,
            source_body_margins=self.source_body_margins,
            source_traffic_planes=self.source_traffic_planes)

    def source_body_margins(self, meter):
        require(self._meter is meter and self._geometry.road is not None,
                'source_admission_before_actual_body_corridor')
        meter.charge('source_body_corridor_reads')
        margins = []
        for row in self.source.rows6[:, :5]:
            values = []
            for pose in (row, _rounded(row)):
                values.append(meter.call('source_body_clearance_map_queries',
                                        self._geometry.road.witness_margin, pose))
            require(all(np.isfinite(value) and value > 0. for value in values),
                    'positive_raw_and_recorder_source_body_clearance_required')
            margins.append(min(values))
        result = frozen_array(margins, (len(self.source.rows6),))
        self.evidence['source_body_corridor'] = dict(poses=len(result),
            minimum_margin_m=float(np.min(result)), query_count=2 * len(result),
            from_current_live_source=True, external_csv_used=False,
            certification_scope='original bitmap source witness only; proposal corridor is affine')
        return result

    def source_traffic_planes(self,meter):
        from control_gt_separation_planes import source_pair_planes, PROPOSAL_GAP_M, NEAR_DISTANCE_M
        require(self._meter is meter,'admitted_native_source_before_GT_planes')
        result=[]
        pairs=0
        for index,row in enumerate(self.source.rows6[:,:5]):
            # Fixed one-second proposal lattice plus the source terminal.
            # Every0.1s source/model/native/Env traffic gate stays mandatory.
            if index % 10 and index != len(self.source.rows6)-1:
                continue
            background,_=self._geometry.background(self.source.ctx,float(row[0]),meter)
            for recorded in (False,True):
                meter.charge('source_GT_plane_pose_variants')
                pose=_rounded(row) if recorded else row
                for actor_id,values in background.items():
                    actor=(dict(values,**{k:round(float(values[k]),4) for k in ('x','y','yaw_rad','v_mps')}) if recorded else values)
                    planes=source_pair_planes(pose,actor,self._geometry.shape,index,actor_id,recorded,meter)
                    pairs+=1
                    if planes:
                        meter.charge('source_GT_plane_retention',len(planes))
                        result.extend(planes)
        self.evidence['source_GT_separation_planes']=dict(from_current_live_native_GT=True,external_CSV_used=False,pairs=pairs,faces=len(result),proposal_gap_m=PROPOSAL_GAP_M,near_distance_m=NEAR_DISTANCE_M,source_indices=len(self.source.rows6),proposal_stride_frames=10,all_actual_point_one_traffic_gates_preserved=True,affine_proposal_only=True,actual_polygon_TTC_side_distance_gates_unchanged=True)
        return tuple(result)
