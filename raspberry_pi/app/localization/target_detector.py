# pylint: disable=C0103,E1101,E1121,R0902,R0903,R0913,R0914,R0917,W0212,W0612
import cv2
from cv2.typing import MatLike
from typing_extensions import override, Buffer
from app.camera.camera_protocol import Camera, Request
from app.dashboard.display_util import DisplayUtil
from app.dashboard.display_protocol import Display
from app.localization.analysis_protocol import ColorAnalysis
from app.localization.detector_base import DetectorBase
from app.network.network_protocol import Network
from app.decoder.decoder_protocol import Decoder
from app.util.timestamps import Timestamps


class TargetDetector(DetectorBase):
    """Detects colored objects."""

    def __init__(
        self,
        cam: Camera,
        display1: Display,
        display2: Display,
        network: Network,
        timestamps: Timestamps,
        analyzer_color: ColorAnalysis,
    ) -> None:
        """
        Note: hue values are 0-180, half the usual range.

        :object_lower: ([H, S, V]) lower bound
        :object_higher: ([H, S, V]) upper bound
        """
        super().__init__(network)
        print("\n*** Interpreter: TargetDetector")

        self._display1 = display1
        self._display2 = display2
        self._network = network
        self._timestamps = timestamps

        self._mtx = cam.get_intrinsic()
        self._dist = cam.get_dist()

        self._analyzer_color = analyzer_color

        # network output for camera FPS
        self._fps = network.get_double_sender("fps")

    @override
    def interpret(self, req: Request) -> None:
        buffer: Buffer
        with req.buffer() as buffer:
            decoder: Decoder = req.decoder()
            img_bgr: MatLike | None = decoder.color(buffer)
            if img_bgr is None:
                return

            if self._network.calibrate():
                # Save the raw image for calibration if requested
                self.write_calibration_image(img_bgr)

            if self._network.undistort_view():
                # Show the undistorted image only if requested.
                self._display2.put(cv2.undistort(img_bgr, self._mtx, self._dist))

            # Capture timestamp in boottime.
            timestamp_boottime_us = req.timestamp_boottime_us()

            # Microsecond age of frame.
            delay_us = Timestamps.delta_us(timestamp_boottime_us)

            # Capture timestamp in servertime.
            servertime: int = self._timestamps.boot_time_to_server_time(
                timestamp_boottime_us
            )

            self._analyzer_color.analyze_color(img_bgr, img_bgr, servertime)

            # Send camera FPS to network.
            fps: float = req.fps()
            self._fps.send(fps)
            self.log_temperature()

            # must flush!  otherwise 100ms update rate.
            self._network.flush()

            # Do the drawing after the NT payload is written to minimize latency.
            # This is not particularly fast or important for prod.
            DisplayUtil.text(img_bgr, f"FPS {fps:2.0f}", (5, 5), 0)
            DisplayUtil.text(img_bgr, f"delay (ms) {delay_us/1000:2.0f}", (5, 5), 1)
            self._display1.put(img_bgr)
