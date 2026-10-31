"""Clock and validity semantics shared by the stream and future ROS consumers."""

from dataclasses import dataclass
import math


def age_ms(now, acquired_at, sequence):
    if not sequence or not math.isfinite(acquired_at) or not 0 <= acquired_at <= now:
        return None
    return round((now - acquired_at) * 1000, 1)


@dataclass
class Observation:
    sequence: int = 0
    source_sequence: int = 0
    acquired_monotonic_s: float = 0.
    acquired_unix_s: float = 0.
    completed_monotonic_s: float = 0.
    error: str | None = None
    errors: int = 0
    busy_since: float | None = None

    def finish(self, source, acquired, wall, completed, error=None):
        self.sequence += 1
        self.source_sequence = source
        self.acquired_monotonic_s = acquired
        self.acquired_unix_s = wall
        self.completed_monotonic_s = completed
        self.error = error
        self.errors += bool(error)
        self.busy_since = None

    def state(self, now, enabled, capture, timeout_s, has_target):
        age = age_ms(now, self.acquired_monotonic_s, self.source_sequence)
        if not enabled:
            status = 'disabled'
        elif not capture['valid']:
            status = 'camera_' + capture['status']
        elif self.busy_since is not None and now - self.busy_since >= timeout_s:
            status = 'timeout'
        elif self.error:
            status = ('timeout' if self.error == 'processing_timeout' else
                      'waiting' if self.error == 'capture_interrupted' else 'error')
        elif not self.source_sequence:
            status = 'waiting'
        elif age is None or now - self.acquired_monotonic_s >= timeout_s:
            status = 'stale'
        else:
            status = 'ok' if has_target else 'no_target'
        valid = status in ('ok', 'no_target')
        return {
            'status': status, 'valid': valid, 'has_target': valid and has_target,
            'sequence': self.sequence, 'source_sequence': self.source_sequence,
            'acquired_monotonic_s': self.acquired_monotonic_s if self.source_sequence else None,
            'acquired_unix_s': self.acquired_unix_s if self.source_sequence else None,
            'completed_monotonic_s': self.completed_monotonic_s if self.sequence else None,
            'age_ms': age,
            'processed_ms': round((self.completed_monotonic_s-self.acquired_monotonic_s)*1000, 1) if self.sequence else None,
            'timeout_ms': timeout_s*1000, 'error': self.error, 'errors': self.errors,
        }
