# pylint: disable=E0611,E1101,R0902,R0903,R0913,R0914,R0917,W0212,W0611
import os
from typing import Any
import cv2
from cv2.typing import MatLike
import ntcore
import numpy as np
from numpy.typing import NDArray
from robotpy_apriltag import AprilTagDetection, AprilTagDetector, AprilTagPoseEstimator
from typing_extensions import override, Buffer
from wpimath.geometry._geometry import Transform3d
from app.camera.camera_protocol import Camera, Request, Size
from app.config.identity import Identity
from app.dashboard.display_protocol import Display
from app.dashboard.display_util import DisplayUtil
from app.decoder.decoder_protocol import Decoder
from app.interpreter.interpreter_protocol import Interpreter
from app.localization.detector_util import DetectorUtil
from app.network.network_protocol import Network
from app.network.structs import Blip, BlipWithCorners
from app.util.timestamps import Timestamps


class TagDetector(Interpreter):
    """A wrapper for the AprilTag detector."""

    IMAGE_DIR = "images"

    def __init__(
        self,
        identity: Identity,
        cam: Camera,
        display1: Display,
        display2: Display,
        network: Network,
        timestamps: Timestamps,
    ) -> None:
        """Finds Apriltags.

        :identity: chooses tag size
        :cam: camera implementation
        :display1: shows annotated image
        :display2: optionally shows undistorted image
        :timestamps: timing source.
        """
        self._identity = identity
        self._cam = cam
        self._display1 = display1
        self._display2 = display2
        self._network = network
        self._timestamps = timestamps

        print("\n*** Interpreter: TagDetector")

        self._mtx: NDArray[np.float32] = cam.get_intrinsic()
        self._dist: NDArray[np.float32] = cam.get_dist()
        print("\n*** intrinsic:\n", self._mtx)
        print("\n*** distortion:", self._dist)

        size: Size = cam.get_size()
        self._width: int = size.width
        self._height: int = size.height

        self._y_len = self._width * self._height

        self._at_detector = AprilTagDetector()
        config = self._at_detector.Config()
        # some of the detection steps can be done in parallel; this
        # should be the same as the number of cores on the machine,
        # which for the Raspberry Pi 5 is four.
        config.numThreads = 4
        # sharpening kernel is [0,-1,0;-1,4,-1;0,-1,0], see apriltag.c
        # this makes adjacent pixels more different.
        config.decodeSharpening = 0.0
        # this doesn't make much difference in FPS
        # config.decodeSharpening = 0.25

        # do not decimate: improves far-away (small) detections
        # config.quadDecimate = 1.0
        # this makes a huge difference in FPS
        # 1.0: 13 FPS
        # 2.0: 40 FPS
        # 4.0: 60 FPS
        # config.quadDecimate = 4.0
        config.quadDecimate = 2.0
        config.refineEdges = True

        # stddev of the blur kernel in pixels: seems to help with small tags
        config.quadSigma = 0.0

        # Write detector debug images to local storage.  Very slow.
        # config.debug = True

        self._at_detector.setConfig(config)
        qtp = self._at_detector.QuadThresholdParameters()
        # The apriltag default is 5.  WPI overrides this
        # with 300, which prevents far-away detections.
        # So set it back to 5.
        # it seems not to make much difference in FPS
        # qtp.minClusterPixels = 300
        qtp.minClusterPixels = 5
        self._at_detector.setQuadThresholdParameters(qtp)
        self._at_detector.addFamily("tag36h11")

        if identity == Identity.DIST_TEST:
            # the distortion rig uses a 33 mm, 20% scale, tag.
            tag_size = 0.033
        elif identity == Identity.FUNNEL:
            # for rotation testing rig
            tag_size = 0.059
        else:
            # normal tag size is 6.5 inches
            tag_size = 0.1651

        print("\n*** tag size:", tag_size)

        self._estimator = AprilTagPoseEstimator(
            AprilTagPoseEstimator.Config(
                tag_size,
                self._mtx[0, 0],
                self._mtx[1, 1],
                self._mtx[0, 2],
                self._mtx[1, 2],
            )
        )

        # network output for tag sightings
        self._blips = network.get_blip_sender()
        self._blips_with_corners = network.get_blip_with_corners_sender()

        # network output for camera FPS
        self._fps = network.get_double_sender("fps")
        self._temp = network.get_double_sender("temp")

        # to keep track of images to write
        self.img_ts_sec: int = 0

        # Make a place to put example images.
        if not os.path.exists(TagDetector.IMAGE_DIR):
            os.mkdir(TagDetector.IMAGE_DIR)

    @override
    def analyze(self, req: Request) -> None:
        buffer: Buffer
        with req.buffer() as buffer:
            decoder: Decoder = req.decoder()
            img: MatLike | None = decoder.mono(buffer)
            if img is None:
                return
            
            if self._network.calibrate():
                # Save the raw image for calibration if requested
                self.write_calibration_image(img)
                
            if self._network.undistort_view():
                # Show the undistorted image only if requested.
                self._display2.put(cv2.undistort(img, self._mtx, self._dist))

            result: list[AprilTagDetection] = self._at_detector.detect(img.data)

            # Capture timestamp in boottime.
            timestamp_boottime_us = req.timestamp_boottime_us()

            # Microsecond age of frame.
            delay_us = Timestamps.delta_us(timestamp_boottime_us)

            # Capture timestamp in servertime.
            servertime: int = self._timestamps.boot_time_to_server_time(
                timestamp_boottime_us
            )

            self.show_and_send_detections(img, result, servertime)

            # Send camera FPS to network.
            fps: float = req.fps()
            self._fps.send(fps)

            # Must flush!  Otherwise, 100ms update rate.
            self._network.flush()

            self.log_temperature()

            # Do the drawing after the NT payload is written to minimize latency.
            # This is not particularly fast or important for prod.

            DisplayUtil.text(img, f"FPS {fps:2.0f}", (10, 80))
            DisplayUtil.text(img, f"DELAY (ms) {delay_us/1000:2.0f}", (10, 160))
            self._display1.put(img)

    def show_and_send_detections(
        self,
        img: MatLike,
        result: list[AprilTagDetection],
        servertime: int,
    ) -> None:
        blips: list[Blip] = []
        blips_with_corners: list[BlipWithCorners] = []
        tag: AprilTagDetection
        for tag in result:
            if tag.getHamming() > 0:
                continue

            # Extract raw (x,y) corners from the tag.
            raw_corners: tuple[
                float, float, float, float, float, float, float, float
            ] = DetectorUtil.raw_corners(tag)

            # Undistort the corners.
            undistorted_corners: tuple[
                float, float, float, float, float, float, float, float
            ] = DetectorUtil.undistorted_corners(self._mtx, self._dist, raw_corners)

            # Redo the homography using the undistorted corners.
            homography: tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any] = (
                DetectorUtil.homography(undistorted_corners)
            )

            # Estimate the pose.
            pose: Transform3d = self._estimator.estimate(
                homography, undistorted_corners
            )
            blips.append(Blip(servertime, tag.getId(), pose))
            blips_with_corners.append(
                BlipWithCorners.make(servertime, tag.getId(), raw_corners, pose)
            )
            DisplayUtil.tag(img, tag, pose)

            # Send sightings to network.
        self._blips.send(blips)
        self._blips_with_corners.send(blips_with_corners)

    def log_temperature(self) -> None:
        """Log the CPU temperature in Celsius.

        The raspberry pi throttles at 80 C.
        """
        try:
            with open(
                "/sys/class/thermal/thermal_zone0/temp", "r", encoding="ascii"
            ) as f:
                raw_temp: int = int(f.read().strip())
                temp_c: float = raw_temp / 1000
                self._temp.send(temp_c)
        except IOError:
            # This file does not exist (e.g. on Windows etc)
            pass

    def write_calibration_image(self, img: NDArray[np.uint8]) -> None:
        """Write an image for later analysis (e.g. calibration).

        WARNING! This is VERY VERY SLOW, like 1 FPS.

        To retrieve these files, use, e.g.:

        scp pi@10.1.0.11:images/* .

        Note the single dot at the end of the line above, which means "current directory"

        These will accumulate forever, so remember to clean it out:

        ssh pi@10.1.0.11 "rm images/img*"
        """
        now_us: int = ntcore._now()
        now_s: int = now_us // 1000000  # once per second
        if now_s > self.img_ts_sec:
            self.img_ts_sec = now_s
            filename: str = TagDetector.IMAGE_DIR + "/img" + str(now_s) + ".png"
            cv2.imwrite(filename, img)
