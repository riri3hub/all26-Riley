"""A detector that does nothing but publish its input."""

# pylint: disable=R0903

import numpy as np
from numpy.typing import NDArray
from typing_extensions import override

from app.camera.camera_protocol import Camera, Request, Size
from app.camera.interpreter_protocol import Interpreter
from app.dashboard.display import Display


class NullDetector(Interpreter):
    def __init__(
        self,
        cam: Camera,
        display: Display,
    ) -> None:
        self.display = display
        print("\n*** Interpreter: NullDetector")
        size: Size = cam.get_size()
        self.width: int = size.width
        self.height: int = size.height
        self.y_len = self.width * self.height

    @override
    def analyze(self, req: Request) -> None:
        with req.yuv() as buffer:
            img: NDArray[np.uint8] = np.frombuffer(
                buffer, dtype=np.uint8, count=self.y_len
            )
            img = img.reshape((self.height, self.width))  # type:ignore
            fps = req.fps()
            delay_us = req.delay_us()
            self.display.text(img, f"FPS {fps:2.0f}", (10, 80))
            self.display.text(img, f"DELAY (ms) {delay_us/1000:2.0f}", (10, 160))
            self.display.put(img)
