"""缓存已认证全源时间域的最坏最终检查预留；字典变化时重新认证。

CPython3.10/64位的字典版本标记可在O(1)时间检测任何顶层字典变更。
布局或解释器不匹配时返回None，调用方沿用原完整覆盖扫描，不复用证明。
只读取经过类型/布局检查的实际dict对象头；不改变dict或GT数据。
"""

import ctypes
import struct
import sys

from control_full_trace_qp_contract import require


class _DictHeader(ctypes.Structure):
    _fields_ = [('reference_count', ctypes.c_ssize_t), ('type_address', ctypes.c_void_p),
                ('used', ctypes.c_ssize_t), ('version', ctypes.c_uint64),
                ('keys_address', ctypes.c_void_p), ('values_address', ctypes.c_void_p)]


def dict_mutation_token(mapping):
    if (type(mapping) is not dict or sys.implementation.name != 'cpython'
            or sys.version_info[:2] != (3, 10) or struct.calcsize('P') != 8):
        return None
    header = _DictHeader.from_address(id(mapping))
    if (header.type_address != id(dict) or header.used != len(mapping)
            or not header.keys_address or header.version == 0):
        return None
    return id(mapping), int(header.version), header.keys_address, header.values_address


class CertifiedReserveCache:
    """Cache an original full-domain proof, keeping its conservative max bound.

    The actor trajectory table must retain its identity/version. Memo changes
    invalidate its coverage proof and trigger the original full-domain loader.
    Every real version read and cache access is charged before it executes.
    """

    def __init__(self, maximum_steps, loader, memo, actors, meter):
        self.maximum_steps = maximum_steps
        self.loader, self.memo, self.actors, self.meter = loader, memo, actors, meter
        self.bound = None
        self.memo_token = self.actor_token = None
        self.loads = self.hits = self.fallbacks = 0

    def get(self, steps):
        require(type(steps) is int and 1 <= steps <= self.maximum_steps,
                'cached_final_step_domain_exceeded')
        meter = self.meter()
        require(meter is not None, 'cached_reserve_requires_admitted_shared_meter')
        meter.charge('certified_reserve_cache_reads')
        meter.charge('certified_reserve_dict_version_reads', 2)
        memo_token = dict_mutation_token(self.memo())
        actor_token = dict_mutation_token(self.actors())
        if memo_token is None or actor_token is None:
            self.fallbacks += 1
            return self.loader(steps)
        require(self.actor_token is None or actor_token == self.actor_token,
                'certified_actor_table_identity_or_version_changed')
        if self.bound is not None and memo_token == self.memo_token:
            self.hits += 1
            return self.bound
        # A refresh certifies every source time again, so a shorter request
        # cannot accidentally certify a later larger request.
        bound = self.loader(self.maximum_steps)
        require(type(bound) is int and bound >= 0, 'certified_final_bound_not_exact_integer')
        meter.charge('certified_reserve_dict_version_reads', 2)
        after_memo = dict_mutation_token(self.memo())
        after_actor = dict_mutation_token(self.actors())
        require(memo_token == after_memo and actor_token == after_actor,
                'final_coverage_loader_changed_certified_dictionary')
        self.bound, self.memo_token, self.actor_token = bound, memo_token, actor_token
        self.loads += 1
        return self.bound

    def summary(self):
        return dict(full_domain_loads=self.loads, cache_hits=self.hits,
                    uncached_fallbacks=self.fallbacks, max_steps=self.maximum_steps,
                    cached_conservative_bound=self.bound,
                    actor_schema_version_guard=True, memo_change_requires_full_refresh=True,
                    unsupported_interpreter_uses_original_scans=True)
