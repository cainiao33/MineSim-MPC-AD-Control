"""F24只缓存认证过的预留；原pose/最终原生物理门逐字继承。"""

from control_certified_reserve_cache import CertifiedReserveCache
from control_full_trace_qp_backend import FullTraceQPBackendAdapter


class FullTraceSignedTurnBackendAdapter(FullTraceQPBackendAdapter):
    def __init__(self, source, datasets_dir):
        super().__init__(source, datasets_dir)
        self._certified_reserve = CertifiedReserveCache(len(source.commands),
            super().final_charge_bound, lambda: self.source.ctx._memo,
            lambda: self.source.ctx.traj, lambda: self._meter)

    def final_charge_bound(self, n_steps):
        bound = self._certified_reserve.get(n_steps)
        self.evidence['final_reserve_cache'] = self._certified_reserve.summary()
        return bound
