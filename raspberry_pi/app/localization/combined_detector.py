"""A combined detector for AprilTags and colored objects (notes) using BGR image only."""

# pylint: disable=C0103,E0611,E1101,E1121,R0902,R0903,R0913,R0914,R0917,W0212,W0612


import cv2
import numpy as np
from numpy.typing import NDArray
import ntcore
from robotpy_apriltag import AprilTagDetection, AprilTagDetector, AprilTagPoseEstimator
from typing_extensions import override
from wpimath.geometry import Rotation3d
from cv2.typing import MatLike, Moments
from app.camera.camera_protocol import Camera, Request, Size
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.display import Display
from app.network.structs import Blip, Target
from app.network.network_protocol import Network


class CombinedDetector(Interpreter):
    def __init__(
        self,
        identity: Identity,
        cam: Camera,
        display: Display,
        network: Network,
        object_lower: NDArray[np.int32],
        object_higher: NDArray[np.int32],
    ) -> None:
        """
        Parameters:
        - object_lower and object_higher: HSV bounds for object detection ([H, S, V])
          Note: hue values are 0-180, half the usual range.
        """
        self._identity = identity
        self._cam = cam
        self._display = display
        self._network = network

        print("\n*** Interpreter: CombinedDetector")

        self._mtx: NDArray[np.float32] = cam.get_intrinsic()
        self._dist: NDArray[np.float32] = cam.get_dist()

        size: Size = cam.get_size()
        self._width: int = size.width
        self._height: int = size.height

        # AprilTag detector setup
        self._at_detector = AprilTagDetector()
        config = self._at_detector.Config()
        config.numThreads = 4
        self._at_detector.setConfig(config)
        self._at_detector.addFamily("tag36h11")

        if identity == Identity.DIST_TEST:
            tag_size = 0.033  # 33 mm for distortion rig
        else:
            tag_size = 0.1651  # normal tag size (6.5 inches)

        self._estimator = AprilTagPoseEstimator(
            AprilTagPoseEstimator.Config(
                tag_size,
                self._mtx[0, 0],
                self._mtx[1, 1],
                self._mtx[0, 2],
                self._mtx[1, 2],
            )
        )

        # Object detection setup
        self._object_lower = object_lower
        self._object_higher = object_higher

        # network output for tag sightings
        self._blips = network.get_blip_sender()

        # network output for target sightings
        self._targets = network.get_target_sender()

    def undistort_points(self, pointlist: list[list[int]]) -> MatLike:
        """Undistort image points using camera matrix and distortion coefficients."""
        points: NDArray[np.float32] = np.array(pointlist, dtype=np.float32).reshape(
            -1, 1, 2
        )
        undistorted = cv2.undistortImagePoints(points, self._mtx, self._dist)
        return undistorted.reshape(-1, 2)

    def detect_tags(
        self,
        img_bgr: NDArray[np.uint8],
        img_display: NDArray[np.uint8],
        servertime: int,
    ) -> None:
        """Detect AprilTags in a BGR image by converting to grayscale internally."""
        # Convert BGR to grayscale for tag detection
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        result: list[AprilTagDetection] = self._at_detector.detect(img_gray.data)
        blips: list[Blip] = []

        for result_item in result:
            if result_item.getHamming() > 0:
                continue

            # Undistort tag corners
            corners: tuple[float, float, float, float, float, float, float, float] = (
                result_item.getCorners((0, 0, 0, 0, 0, 0, 0, 0))
            )
            pairs = np.reshape(corners, [4, 2])
            pairs = cv2.undistortImagePoints(pairs, self._mtx, self._dist)

            # Reformat corners for pose estimation
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

            homography = result_item.getHomography()
            pose = self._estimator.estimate(homography, corners)
            blips.append(Blip(servertime, result_item.getId(), pose))
            self._display.tag(img_display, result_item, pose)  # Display on BGR image

        self._blips.send(blips)

    def detect_objects(
        self,
        img_bgr: NDArray[np.uint8],
        img_display: NDArray[np.uint8],
        servertime: int,
    ) -> None:
        """Detect colored objects in a BGR image."""
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)

        # Threshold for object color
        img_range = cv2.inRange(img_hsv, self._object_lower, self._object_higher)

        # Clean up the mask
        floodfill = img_range.copy()
        mask = np.zeros((self._height + 2, self._width + 2), np.uint8)
        cv2.floodFill(floodfill, mask, [0, 0], [255])
        # floodfill_inv = cv2.bitwise_not(floodfill)
        # img_floodfill = cv2.bitwise_or(img_range, floodfill_inv)
        median = cv2.medianBlur(img_range, 5)

        # Find contours
        contours, _ = cv2.findContours(median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        targets: list[Target] = []
        points_to_undistort: list[list[int]] = []
        contour_info: list[tuple[MatLike, int, int]] = []

        for contour in contours:
            # https://en.wikipedia.org/wiki/Image_moment
            mmnts: Moments = cv2.moments(contour)

            if mmnts["m00"] == 0:
                continue

            cY = int(mmnts["m01"] / mmnts["m00"])
            if mmnts["m00"] < 500 or mmnts["m00"] > (
                (30000) * ((self._height / 2) / (self._height - cY + 1))
            ):  # Minimum contour area
                continue
                # calculates how big the object is based on how far it is.
            cX = int(mmnts["m10"] / mmnts["m00"])

            points_to_undistort.append([cX, cY])
            contour_info.append((contour, cX, cY))

        if points_to_undistort:
            # Undistort all object centers at once
            undistorted_points = self.undistort_points(points_to_undistort)

            c: MatLike
            orig_cX: int
            orig_cY: int
            undist_cX: float
            undist_cY: float
            for (c, orig_cX, orig_cY), (undist_cX, undist_cY) in zip(
                contour_info, undistorted_points
            ):
                yNormalized = (self._height / 2 - undist_cY) / self._mtx[1, 1]
                xNormalized = (self._width / 2 - undist_cX) / self._mtx[0, 0]

                initial = np.array([1, 0, 0], dtype=np.float64)
                final = np.array([1, xNormalized, yNormalized], dtype=np.float64)
                rotation = Rotation3d(initial=initial, final=final)

                targets.append(Target(servertime, rotation))
                self._display.note(img_display, c, orig_cX, orig_cY)

            # only send if there's anything to say
            self._targets.send(targets)

        # self.display.put(img_range)

    @override
    def analyze(self, req: Request) -> None:
        """Process both tags and objects from the BGR image."""
        with req.rgb() as buffer_rgb:
            # Get BGR image for both detectors
            img_bgr: NDArray[np.uint8] = np.frombuffer(buffer_rgb, dtype=np.uint8)

            img_bgr = img_bgr.reshape((self._height, self._width, 3))
            img_display = img_bgr.copy()

            # microsecond age of frame
            delay_us = req.delay_us()

            # localtime in microseconds
            localtime: int = int(ntcore._now() - delay_us)
            servertime: int = self._network.server_time(localtime)

            # Run both detectors on the BGR image
            self.detect_tags(img_bgr, img_display, servertime)
            self.detect_objects(img_bgr, img_display, servertime)

            # Network flush and display
            self._network.flush()

            fps = req.fps()
            self._display.text(img_display, f"FPS {fps:2.0f}", (5, 65))
            self._display.text(
                img_display, f"delay (ms) {delay_us/1000:2.0f}", (5, 105)
            )
            self._display.put(img_display)
