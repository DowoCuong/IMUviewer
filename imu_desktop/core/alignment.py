"""Bounded quaternion interpolation on a common device clock; never extrapolate."""
from __future__ import annotations

import math
from collections import Counter, deque
from dataclasses import dataclass

from .math3d import Quaternion, slerp
from .protocol import SensorSample, signed_delta
from .quality import EpochTracker,usable_at


@dataclass(frozen=True)
class JointFrame:
    time_us: float
    orientations: dict[int, Quaternion]
    clock_uncertainty_us: float
    interpolation_gap_us: float
    time_basis: str


class SampleSynchronizer:
    def __init__(self, nodes: tuple[int, ...] = (1, 2, 3, 4), rate_hz: float = 100,
                 max_gap_us: float = 30000) -> None:
        if not nodes or len(set(nodes)) != len(nodes) or any(n not in range(1, 5) for n in nodes):
            raise ValueError("Danh sách cảm biến không hợp lệ")
        if not 1 <= rate_hz <= 1000 or not math.isfinite(max_gap_us) or max_gap_us <= 0:
            raise ValueError("Cấu hình ghép mẫu không hợp lệ")
        self.nodes, self.period_us, self.max_gap_us = nodes, 1_000_000/rate_hz, max_gap_us
        self.counts: Counter[str] = Counter()
        self.epochs=EpochTracker()
        self.reset()

    def reset(self) -> None:
        self.buffers: dict[int, deque[SensorSample]] = {n: deque(maxlen=32) for n in self.nodes}
        self.sessions: dict[int, int] = {}
        self.domain: tuple[str, int] | None = None
        self.next_time: float | None = None
        self.generation = getattr(self, "generation", 0)+1

    def add(self, sample: SensorSample) -> list[JointFrame]:
        if sample.node not in self.buffers:
            return []
        changed,stale=self.epochs.observe(sample)
        if stale: self.counts['old_epoch']+=1; return []
        if changed: self.reset(); self.counts['sensor_epoch_resets']+=1
        if sample.time_us is None:
            self.counts["unmapped_clock"] += 1
            return []
        if (sample.node in self.sessions and self.sessions[sample.node] != sample.session) or (
                self.domain is not None and self.domain != sample.domain):
            self.reset()
            self.counts["clock_or_device_resets"] += 1
        self.sessions[sample.node], self.domain = sample.session, sample.domain
        queue = self.buffers[sample.node]
        if not sample.usable or sample.quaternion is None:
            queue.clear()
            self.next_time = None
            self.counts["unusable_samples"] += 1
            return []
        if queue:
            last = queue[-1]
            if sample.sequence == last.sequence:
                self.counts["retransmissions"] += 1
                return []
            if sample.time_us <= last.time_us or signed_delta(sample.sequence, last.sequence) <= 0:
                self.counts["out_of_order"] += 1
                return []
            if sample.time_us-last.time_us > self.max_gap_us:
                queue.clear()
                self.next_time = None
                self.counts["gaps"] += 1
        queue.append(sample)
        if any(not q for q in self.buffers.values()):
            return []
        earliest = max(q[0].time_us for q in self.buffers.values())
        latest = min(q[-1].time_us for q in self.buffers.values())
        if self.next_time is None or self.next_time < earliest:
            self.next_time = math.ceil(earliest/self.period_us)*self.period_us
        result = []
        while self.next_time <= latest:
            values, uncertainty, gap = {}, 0.0, 0.0
            for node, samples in self.buffers.items():
                while len(samples) > 1 and samples[1].time_us <= self.next_time:
                    samples.popleft()
                a = samples[0]
                if not usable_at(a,self.next_time): break
                if a.time_us == self.next_time:
                    values[node] = a.quaternion
                    uncertainty = max(uncertainty, a.clock_uncertainty_us)
                elif len(samples) > 1:
                    b = samples[1]
                    if not usable_at(b,self.next_time): break
                    interval = b.time_us-a.time_us
                    if not 0 < interval <= self.max_gap_us:
                        break
                    values[node] = slerp(a.quaternion, b.quaternion, (self.next_time-a.time_us)/interval)
                    uncertainty = max(uncertainty, a.clock_uncertainty_us, b.clock_uncertainty_us)
                    gap = max(gap, interval)
                else:
                    break
            if len(values) == len(self.nodes):
                result.append(JointFrame(self.next_time, values, uncertainty, gap, self.domain[0]))
            self.next_time += self.period_us
        return result
