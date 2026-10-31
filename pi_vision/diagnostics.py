"""Bounded asynchronous raw-frame evidence from the existing capture owner."""

import json
import logging
from pathlib import Path
import queue
import threading
import time

import cv2


class DiagnosticRecorder:
    def __init__(self, directory, samples=30, interval=1.0):
        if type(samples) is not int or not 1 <= samples <= 300:
            raise ValueError('diagnostic samples must be 1..300')
        if not .2 <= interval <= 60:
            raise ValueError('diagnostic interval must be .2..60 seconds')
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.limit, self.interval = samples, interval
        self._queue = queue.Queue(maxsize=2)
        self._stop = threading.Event()
        self._next_at, self._last_source = 0.0, -1
        self.queued = self.saved = self.dropped = self.errors = 0
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._thread.start()

    def offer(self, frame, metadata):
        now = time.monotonic()
        source = metadata['source_sequence']
        if (self._stop.is_set() or self.queued >= self.limit or now < self._next_at
                or source <= self._last_source):
            return
        # Capture publishes a new array each time; detection never mutates it.
        try:
            self._queue.put_nowait((self.queued + 1, frame, metadata))
        except queue.Full:
            self.dropped += 1
            return
        self.queued += 1
        self._last_source = source
        self._next_at = now + self.interval

    def _write_loop(self):
        while not self._stop.is_set() or not self._queue.empty():
            try:
                number, frame, metadata = self._queue.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                stem = self.directory / f'sample_{number:04d}'
                if not cv2.imwrite(str(stem.with_suffix('.png')), frame):
                    raise OSError('PNG write failed')
                metadata = dict(metadata, image=stem.with_suffix('.png').name)
                stem.with_suffix('.json').write_text(
                    json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
                self.saved += 1
            except Exception:
                self.errors += 1
                logging.getLogger(__name__).exception('Diagnostic write failed')
            finally:
                self._queue.task_done()

    def state(self):
        return {'enabled': True, 'limit': self.limit, 'queued': self.queued,
                'saved': self.saved, 'dropped': self.dropped, 'errors': self.errors}

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
