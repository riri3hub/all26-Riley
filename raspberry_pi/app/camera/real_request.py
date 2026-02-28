# pylint: disable=E0401

from contextlib import AbstractContextManager
from typing import cast
from typing_extensions import Buffer, override

from picamera2 import CompletedRequest  # type: ignore
from picamera2.request import _MappedBuffer  # type: ignore

from app.camera.request_protocol import Request
from app.util.timer import Timer


class RealRequest(Request):
    def __init__(self, req: CompletedRequest, fps: float, rolling: bool):  # type: ignore
        # Before we get a CompletedRequest, its constructor has used the
        # camera allocator sync property to:
        # * instantiate a DMA allocator sync for each buffer
        # * tell the camera allocator to mark the buffers as 'in use'
        # * __enter__() each buffer's DmaSync, which calls ioctl DMA_BUF_SYNC_START
        self._req: CompletedRequest = req
        self._fps = fps
        self._rolling = rolling

    @override
    def fps(self) -> float:
        # a typical value for a real camera
        return self._fps

    @override
    def delay_us(self) -> int:
        metadata = self._req.get_metadata()  # type: ignore
        # Time of first row received, this is roughly the "readout timestamp"
        sensor_timestamp_ns = cast(int, metadata["SensorTimestamp"])

        # Half the exposure time.
        exposure_term_us = cast(int, metadata["ExposureTime"] * 0.5)
        exposure_term_ns = exposure_term_us * 1000

        # Extra constant delay.
        # 2/21/26 using a real robot.  I think this is correcting for
        # roborio loop delay, not just camera delay.
        frame_term_ms = 30
        # 2/20/26 this from the "camera_delay" project
        # frame_term_ms = 2
        frame_term_ns = cast(int, frame_term_ms * 1000000)

        exposure_timestamp_ns = sensor_timestamp_ns - frame_term_ns - exposure_term_ns

        if self._rolling:
            # For a global shutter, the whole frame is exposed at once,
            # so the exposure timestamp applies to all the pixels.
            # For a rolling shutter, rows are exposed over the entire
            # frame duration (1/fps), i.e. *after* the data from the first
            # row is received.  Take the midpoint of this period.
            # TODO: assign a different timestamp to each tag, depending on
            # where it is in the frame -- note since we're moving away from
            # rolling shutters this is maybe not worth worrying about :-)
            frame_duration_us = cast(int, metadata["FrameDuration"])
            frame_duration_ns = frame_duration_us * 1000
            exposure_timestamp_ns += frame_duration_ns // 2

        # The delay is the difference between the exposure time and the current instant.
        delay_ns: int = Timer.time_ns() - exposure_timestamp_ns
        delay_us = delay_ns // 1000

        return delay_us

    @override
    def rgb(self) -> AbstractContextManager[Buffer]:
        return self._buffer("main")

    @override
    def yuv(self) -> AbstractContextManager[Buffer]:
        return self._buffer("lores")

    def _buffer(self, stream: str) -> AbstractContextManager[Buffer]:
        # Returns AbstractContextManager[Buffer] because the flow is:
        #
        # During picamera2.configure(), the DmaAllocator allocates
        # the requested (buffer_count) number of dma buffers, which
        # are mmap.mmap objects, which are Buffers.
        #
        # The camera uses DmaAllocator, whose sync property
        # is the DmaSync constructor.
        #
        # The _MappedBuffer constructor invokes the DmaSync constructor
        # to get a DmaSync.  _MappedBuffer.__enter__() delegates to DmaSync to
        # do the ioctl DMA_BUF_SYNC_START (again), and return the mmap.
        #
        # _MappedBuffer.__exit__() also delegates to DmaSync, which implements
        # DMA_BUF_SYNC_END.
        #
        # since _MappedBuffer implements __enter__ and __exit__, we
        # can duck-type it an AbstractContextManager.
        #
        # Note that when the _MappedBuffer is __exit__'ed, the DMA buffer should
        # not be touched anymore, which means not using numpy views on it.
        # The Picamera code addresses this in CompletedRequest.make_buffer by
        # *copying* the buffer, but we definitely don't want to do that.  Just
        # do all your work on the buffer within the scope of the context manager.
        #
        # This use of _MappedBuffer is not necessary for the buffer-reservation
        # (it's done by the CompletedRequest), but using _MappedBuffer is, by far,
        # the easiest way to get at the mmap buffer.
        #
        # To use the buffer, you can pass it to np.frombuffer().
        return _MappedBuffer(self._req, stream)  # type: ignore

    @override
    def release(self) -> None:
        # Calls DmaSync.__exit__() which invokes ioctl DMA_BUF_SYNC_END
        # to release the DMA buffer, and unmark the app-level 'in use' flag.
        # Note that the _MappedBuffer has already done the ioctl work, so this
        # is redundant.
        self._req.release()  # type: ignore
