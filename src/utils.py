
import time
import threading
from collections import deque


class FramerateCounter:
    '''track framerate using a rolling average over the last `maxlen` frames'''

    def __init__(self, maxlen: 'int | None' = None):
        self._buffer: deque[float] = deque(maxlen=maxlen)
        '''track frames durations'''
        self._last_timestamp: float = 0
        self._acc: float = 0
        '''internal accumulator'''

        self._thread = threading.Thread(
            target=self._run,
            daemon=True
        )

    def start(self):
        self._thread.start()

    def _run(self):
        '''internal thread to track FPS in the case of a loss of signal'''
        while True:
            now = time.time()
            elapsed = now - self._last_timestamp

            if elapsed > .5 and len(self._buffer) > 0:
                self._acc -= self._buffer.popleft()

            time.sleep(.5)

    def new_frame(self) -> None:
        '''add a new timestamp'''
        now = time.time()

        if self._last_timestamp > 0:
            elapsed = now - self._last_timestamp
            if len(self._buffer) == self._buffer.maxlen:
                self._acc -= self._buffer.popleft()
            self._acc += elapsed
            self._buffer.append(elapsed)

        self._last_timestamp = now

    @property
    def avg_fps(self) -> float:
        '''average framerate. -1 if not defined'''
        try:
            return len(self._buffer)/(self._acc + (time.time() - self._last_timestamp))
        except:
            return -1.


class PacketLossCounter:
    '''track packet loss per time unit'''

    def __init__(self, maxlen: 'int | None' = None):
        self._buffer: deque[float] = deque(maxlen=maxlen)
        '''track number of packets lost'''
        self._timebuffer: deque[float] = deque(maxlen=maxlen)
        '''track time'''
        self._acc: float = 0
        '''internal accumulator'''

    def new_report(self, __lost: int) -> None:
        '''add a new packet loss entry'''

        self._timebuffer.append(time.time())

        if len(self._buffer) == self._buffer.maxlen:
            self._acc -= self._buffer.popleft()
        self._acc += __lost
        self._buffer.append(__lost)

    @property
    def avg_loss(self) -> float:
        '''average losses. -1 if not defined'''
        try:
            return self._acc / (self._timebuffer[-1] - self._timebuffer[0])
        except:
            return -1.
