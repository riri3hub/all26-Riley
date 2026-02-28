# pylint: disable=W2301

from typing import Protocol
from contextlib import AbstractContextManager
from typing_extensions import Buffer


class Request(Protocol):
    def fps(self) -> float:
        """FPS calculated from the previous capture."""
        ...

    def delay_us(self) -> int:
        """Duration between the capture instant of the center of the frame
        and the current instant, microseconds"""
        ...

    def rgb(self) -> AbstractContextManager[Buffer]:
        """Context-managed Buffer containing RGB888.
        Remember that when OpenCV says "RGB" it really means "BGR"
        github.com/raspberrypi/picamera2/issues/848"""
        ...

    def yuv(self) -> AbstractContextManager[Buffer]:
        """Context-managed Buffer containing YUV420."""
        ...

    def release(self) -> None: ...
