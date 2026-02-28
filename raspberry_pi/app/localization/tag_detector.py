"""A wrapper for the AprilTag detector."""

# pylint: disable=E0611,E1101,R0902,R0903,R0913,R0914,R0917,W0212

import os
from typing import cast
import cv2
import ntcore
import numpy as np
from numpy.typing import NDArray
from robotpy_apriltag import AprilTagDetection, AprilTagDetector, AprilTagPoseEstimator
from typing_extensions import override
from app.camera.camera_protocol import Camera, Request, Size
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.display import Display
from app.network.network_protocol import Network
from app.network.structs import Blip


class TagDetector(Interpreter):
    IMAGE_DIR = "images"

    def __init__(
        self, identity: Identity, cam: Camera, display: Display, network: Network
    ) -> None:
        """Debug is very slow.  It writes apriltag detector debug images
        into the same filenames over and over, and also writes
        timestamped images for later analysis.
        """
        self._identity = identity
        self._cam = cam
        self._display = display
        self._network = network

        print("\n*** Interpreter: TagDetector")

        self._mtx: NDArray[np.float32] = cam.get_intrinsic()
        self._dist: NDArray[np.float32] = cam.get_dist()

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
        config.quadDecimate = 4.0
        config.refineEdges = True

        # stddev of the blur kernel in pixels: seems to help with small tags
        config.quadSigma = 0.0

        # Write calibration images to local storage.  Very slow.
        if self._network.calibrate():
            config.debug = True

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
        with req.yuv() as buffer:
            # truncate, ignore chrominance. this makes a view, very fast (300 ns)
            img: NDArray[np.uint8] = cast(
                NDArray[np.uint8],
                np.frombuffer(buffer, dtype=np.uint8, count=self._y_len),  # type:ignore
            )

            # this  makes a view, very fast (150 ns)
            img = img.reshape((self._height, self._width))  # type:ignore

            if self._network.calibrate():
                self.write_calibration_image(img)

            # Uncomment this line to undistort the whole image, for debugging.
            # img = undistort(img, self.mtx, self.dist)

            result: list[AprilTagDetection] = self._at_detector.detect(img.data)

            # microsecond age of frame
            delay_us: int = req.delay_us()

            # localtime in microseconds
            localtime: int = int(ntcore._now() - delay_us)
            servertime: int = self._network.server_time(localtime)

            blips: list[Blip] = []
            result_item: AprilTagDetection
            for result_item in result:
                if result_item.getHamming() > 0:
                    continue
                corners = self.tag_corners(result_item)
                homography = result_item.getHomography()
                pose = self._estimator.estimate(homography, corners)
                blips.append(Blip(servertime, result_item.getId(), pose))
                self._display.tag(img, result_item, pose)

            # Send sightings to network.
            self._blips.send(blips)

            # Send camera FPS to network.
            fps: float = req.fps()
            self._fps.send(fps)

            # Must flush!  Otherwise, 100ms update rate.
            self._network.flush()

            self.log_temperature()

            # Do the drawing after the NT payload is written to minimize latency.
            # This is not particularly fast or important for prod.
            self._display.text(img, f"FPS {fps:2.0f}", (10, 80))
            self._display.text(img, f"DELAY (ms) {delay_us/1000:2.0f}", (10, 160))
            self._display.put(img)

    def tag_corners(
        self, result_item: AprilTagDetection
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """Extract tag corners from the detection and 'undistort' them"""

        # UNDISTORT EACH ITEM
        # undistortPoints is at least 10X faster than undistort on the whole image.
        # the order is:
        # lower left, lower right, upper right, upper left
        corners: tuple[float, float, float, float, float, float, float, float] = (
            result_item.getCorners((0, 0, 0, 0, 0, 0, 0, 0))
        )

        # undistortPoints wants [[x0,y0],[x1,y1],...]
        pairs = np.reshape(corners, [4, 2])
        # undistortImagePoints takes [u,v] pixel pairs
        # this is just undistortPoints with mtx as the new intrinsic.
        pairs = cv2.undistortImagePoints(pairs, self._mtx, self._dist)

        # the estimator wants [x0, y0, x1, y1, ...]
        # pairs has an extra dimension, so redo it:
        corners = (
            pairs[0][0][0],
            pairs[0][0][1],
            pairs[1][0][0],
            pairs[1][0][1],
            pairs[2][0][0],
            pairs[2][0][1],
            pairs[3][0][0],
            pairs[3][0][1],
        )
        return corners

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

        To retrieve these files, use, e.g.:

        scp pi@10.1.0.11:images/* .

        These will accumulate forever, so remember to clean it out:

        ssh pi@10.1.0.11 "rm images/img*"
        """
        now_us: int = ntcore._now()
        now_s: int = now_us // 1000000  # once per second
        if now_s > self.img_ts_sec:
            self.img_ts_sec = now_s
            filename: str = TagDetector.IMAGE_DIR + "/img" + str(now_s) + ".png"
            cv2.imwrite(filename, img)
