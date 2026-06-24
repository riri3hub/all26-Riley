import time
import ntcore
from threading import Event

from queue import Queue, Empty

from app.framework.looper import Looper
from typing_extensions import override


class DriftLoop(Looper):
    """Consumes timing offset estimates from the queue."""

    def __init__(
        self, inst: ntcore.NetworkTableInstance, queue: Queue, done: Event
    ) -> None:
        super().__init__(done)
        print("*** DriftLoop")
        self._inst = inst
        self._queue = queue
        self._first_estimate = 0
        # we could use the offset in, say, network writers
        self._offset = 0
        self._offset_pub = self._inst.getIntegerTopic("offset (us)").publish()
        self._drift_pub = inst.getIntegerTopic("drift (us)").publish()

    @override
    def execute(self) -> None:
        try:
            self._offset: int = self._queue.get_nowait()
            
            if self._first_estimate == 0:
                self._first_estimate = self._offset

            if abs(self._first_estimate - self._offset) > 1000000:
                # this is for when we restart the rio
                self._first_estimate = self._offset

            drift = self._offset - self._first_estimate
            self._offset_pub.set(self._offset)
            self._drift_pub.set(drift)
        except Empty:
            pass
            # nothing to do

        time.sleep(0.02)

    @override
    def end(self) -> None:
        pass
