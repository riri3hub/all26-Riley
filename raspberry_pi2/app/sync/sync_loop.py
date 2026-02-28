import time
import ntcore

from threading import Event

from app.framework.looper import Looper
from typing_extensions import override
from queue import Queue, Full

from app.network.network import SyncRequest, SyncReply


class SyncLoop(Looper):
    """Fills the queue with timing offset estimates."""

    def __init__(
        self, inst: ntcore.NetworkTableInstance, queue: Queue, done: Event
    ) -> None:
        super().__init__(done)
        print("*** SyncLoop")
        self._inst = inst
        self._queue = queue
        self._offset: int = 0
        self._request_pub = inst.getStructTopic("syncrequest", SyncRequest).publish()
        self._reply_sub = inst.getStructTopic("syncreply", SyncReply).subscribe(
            SyncReply(0, 0, 0)
        )

    @override
    def execute(self) -> None:
        self._request_pub.set(SyncRequest(ntcore._now()))

        replies: list[ntcore.TimestampedStruct] = self._reply_sub.readQueue()
        if replies:
            syncreply: SyncReply = replies[-1].value
            now = ntcore._now()
            measurement = self.offset(syncreply.org, syncreply.rec, syncreply.xmt, now)
            self._offset = self.fuse(self._offset, measurement)
            try:
                self._queue.put_nowait(self._offset)
            except Full:
                print("*** queue is full, which should never happen ***")

        self._inst.flush()
        # avoid spinning too fast
        time.sleep(0.02)

    @override
    def end(self) -> None:
        pass

    @staticmethod
    def offset(org: int, rec: int, xmt: int, dst: int) -> int:
        return (rec + xmt - dst - org) // 2

    @staticmethod
    def fuse(estimate: int, measurement: int) -> int:
        diff = estimate - measurement
        if abs(diff) > 30000:
            # more than 30 ms off, go right to the measurement
            # note the clock granularity leads to about 20 ms of jitter
            return measurement
        else:
            # very gentle averaging, seems to yield
            # about 1 ms of noise
            return (estimate * 999 + measurement) // 1000
