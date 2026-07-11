# pylint: disable=C0103,E0611,E1101,E1121,R0902,R0903,R0913,R0914,R0917,W0212,W0612

import cv2
import numpy as np

from numpy.typing import NDArray
from typing_extensions import override
from app.camera.camera_protocol import Camera, Request
from app.dashboard.display_protocol import Display
from app.dashboard.display_util import DisplayUtil
from app.decoder.decoder_protocol import Decoder
from app.localization.analysis_protocol import ColorAnalysis, MonoAnalysis
from app.localization.detector_base import DetectorBase
from app.network.network_protocol import Network
from app.util.timestamps import Timestamps


class CombinedDetector(DetectorBase):
    """A combined detector for AprilTags and colored objects."""

    def __init__(
        self,
        cam: Camera,
        display1: Display,
        display2: Display,
        network: Network,
        timestamps: Timestamps,
        analyzer_mono: MonoAnalysis,
        analyzer_color: ColorAnalysis,
    ) -> None:
        super().__init__(network)
        print("\n*** Interpreter: CombinedDetector")

        self._display1 = display1
        self._display2 = display2
        self._network = network
        self._timestamps = timestamps

        self._mtx: NDArray[np.float32] = cam.get_intrinsic()
        self._dist: NDArray[np.float32] = cam.get_dist()

        self._analyzer_mono = analyzer_mono
        self._analyzer_color = analyzer_color

        # network output for camera FPS
        self._fps = network.get_double_sender("fps")

    @override
    def interpret(self, req: Request) -> None:
        """Process both tags and objects from the BGR image."""
        with req.buffer() as buffer:
            #
            # Compute delay and servertime
            #
            # Capture timestamp in boottime.
            timestamp_boottime_us = req.timestamp_boottime_us()

            # Microsecond age of frame.
            delay_us = Timestamps.delta_us(timestamp_boottime_us)

            # Capture timestamp in servertime.
            servertime: int = self._timestamps.boot_time_to_server_time(
                timestamp_boottime_us
            )

            decoder: Decoder = req.decoder()

            img_bgr = None
            img_mono = None

            # Image for analysis, do not modify.
            img_bgr = decoder.color(buffer)
            if img_bgr is None:
                return

            if self._network.calibrate():
                # Save the raw image for calibration if requested
                self.write_calibration_image(img_bgr)

            if self._network.undistort_view():
                # Show the undistorted image only if requested.
                self._display2.put(cv2.undistort(img_bgr, self._mtx, self._dist))

            # Convert BGR to grayscale for tag detection.
            img_mono = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            img_mono = np.ascontiguousarray(img_mono)

            # Image for display, with annotations.
            # prefer the color one if it exists, otherwise use the mono one.
            img_display = (img_bgr or img_mono).copy()

            # tag detection uses the mono image.
            if self._analyzer_mono:
                self._analyzer_mono.analyze_mono(img_mono, img_display, servertime)

            # object detection uses the color image.
            if self._analyzer_color:
                self._analyzer_color.analyze_color(img_bgr, img_display, servertime)

            # Send camera FPS to network.
            fps: float = req.fps()
            self._fps.send(fps)
            self.log_temperature()

            # Must flush!  Otherwise, 100ms update rate.
            self._network.flush()

            # Do the drawing after the NT payload is written to minimize latency.
            # This is not particularly fast or important for prod.
            DisplayUtil.text(img_display, f"FPS {fps:2.0f}", (5, 5), 0)
            DisplayUtil.text(img_display, f"delay (ms) {delay_us/1000:2.0f}", (5, 5), 1)
            self._display1.put(img_display)
