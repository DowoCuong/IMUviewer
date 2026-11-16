"""Disk-backed rows. Acquisition and export never materialize session history."""
from __future__ import annotations
import json
import tempfile
import threading
from collections import Counter


class RowLog:
    def __init__(self):
        self.file = tempfile.TemporaryFile(mode='w+b')
        self.lock = threading.RLock()
        self.count = 0
        self.minimum, self.maximum = {}, {}

    def append(self, row):
        encoded = (json.dumps(row, allow_nan=False, separators=(',', ':'))+'\n').encode()
        with self.lock:
            self.file.seek(0, 2)
            self.file.write(encoded)
            self.count += 1
            for key, value in row.items():
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    self.minimum[key] = min(self.minimum.get(key, value), value)
                    self.maximum[key] = max(self.maximum.get(key, value), value)

    def __len__(self):
        return self.count

    def __bool__(self):
        return self.count != 0

    def __iter__(self):
        with self.lock:
            self.file.seek(0, 2)
            end = self.file.tell()
        position = 0
        while position < end:
            with self.lock:
                self.file.seek(position)
                line = self.file.readline()
                position = self.file.tell()
            if not line:
                raise OSError('Tệp ghi góc bị gián đoạn')
            yield json.loads(line)

    def close(self):
        self.file.close()


class Coverage:
    """Count valid 10 ms ticks per component over the observed hub interval."""
    def __init__(self):
        self.first = self.last = None
        self.valid = Counter()
        self.reasons = Counter()

    def add(self, time_us, angles, quality):
        self.first = time_us if self.first is None else min(self.first, time_us)
        self.last = time_us if self.last is None else max(self.last, time_us)
        for key, value in angles.items():
            if value is not None:
                self.valid[key] += 1
            else:
                self.reasons[key+':'+quality.get(key, 'unavailable')] += 1

    def snapshot(self):
        ticks = 0 if self.first is None else int(round((self.last-self.first)/10000))+1
        return {'observed_hub_ticks': ticks, 'valid_ticks': dict(self.valid),
                'valid_fraction': {k: v/ticks if ticks else None for k, v in self.valid.items()},
                'explicit_invalid_reasons': dict(self.reasons),
                'note': 'Missing ticks also reduce coverage; RF loss is not inferred.'}
