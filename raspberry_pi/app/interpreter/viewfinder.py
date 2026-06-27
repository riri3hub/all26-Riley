# pylint: disable=C0103,E1101,R0903
from cv2.typing import MatLike
from typing_extensions import Buffer, override
from app.camera.camera_protocol import Request
from app.dashboard.display import Display
from app.decoder.decoder_protocol import Decoder
from app.interpreter.interpreter_protocol import Interpreter
from app.network.network_protocol import Network
from app.util.timestamps import Timestamps


class Viewfinder(Interpreter):
    """A detector that does nothing but publish its input."""

    def __init__(
        self,
        display: Display,
        network: Network,
    ) -> None:
        print("\n*** Interpreter: NullDetector")
        self._display = display
        # network output for camera FPS
        self._fps = network.get_double_sender("fps")


    @override
    def analyze(self, req: Request) -> None:
        buffer: Buffer
        with req.buffer() as buffer:
            decoder: Decoder = req.decoder()
            img_bgr: MatLike | None = decoder.color(buffer)
            if img_bgr is None:
                return
            fps = req.fps()
            self._fps.send(fps)
            delay_us = Timestamps.delta_us(req.timestamp_boottime_us())
            self._display.text(img_bgr, f"FPS {fps:2.0f}", (5, 65))
            self._display.text(img_bgr, f"delay (ms) {delay_us/1000:2.0f}", (5, 105))
            self._display.put(img_bgr)
