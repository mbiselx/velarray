
import time
from collections import deque

from rospy import Time


class FramerateCounter:
    '''track framerate using a rolling average over the last `maxlen` frames'''

    def __init__(self, maxlen: 'int | None' = None):
        self._buffer: deque[float] = deque(maxlen=maxlen)
        self._last_timestamp: 'float | None' = None
        self._acc: float = 0

    def new_timestamp(self, __ts: 'float | Time') -> None:
        '''add a new timestamp'''
        if isinstance(__ts, Time):
            __ts = __ts.to_sec()

        if self._last_timestamp is not None:
            duration = __ts - self._last_timestamp
            if len(self._buffer) == self._buffer.maxlen:
                self._acc -= self._buffer[0]
            self._acc += duration
            self._buffer.append(duration)

        self._last_timestamp = __ts

    @property
    def fps(self) -> float:
        '''instantaneous framerate. -1 if not defined'''
        try:
            return 1/self._buffer[-1]
        except:
            return -1.

    @property
    def avg_fps(self) -> float:
        '''average framerate. -1 if not defined'''
        try:
            return len(self._buffer)/self._acc
        except:
            return -1.
