"""A wrapper for the note detector."""

# pylint: disable=C0103,E1101,E1121,R0902,R0903,R0913,R0914,R0917,W0612

import cv2
import numpy as np
from numpy.typing import NDArray
import ntcore
from wpimath.geometry import Rotation3d
from cv2.typing import MatLike, Moments
from typing_extensions import Buffer

from app.camera.camera_protocol import Camera, Request, Size
from app.camera.interpreter_protocol import Interpreter
from app.dashboard.display import Display
from app.network.network_protocol import Network
from app.network.structs import Target


class TargetDetector(Interpreter):
    def __init__(
        self,
        cam: Camera,
        display: Display,
        network: Network,
        object_lower: NDArray[np.int32],
        object_higher: NDArray[np.int32],
    ) -> None:
        """
        object_lower and object_higher are ([H, S, V]) bounds.
        NOTE: hue values are 0-180, half the usual range.
        """
        self._cam = cam
        self._display = display
        self._network = network
        print("\n*** Interpreter: NoteDetector")

        self._mtx = self._cam.get_intrinsic()
        self._dist = self._cam.get_dist()

        size: Size = cam.get_size()
        self._width: int = size.width
        self._height: int = size.height

        self._object_lower = object_lower
        self._object_higher = object_higher

        # network output for target sightings
        self._targets = network.get_target_sender()

    def undistort_points(self, pointlist: list[list[int]]) -> MatLike:
        """Undistort image points using camera matrix and distortion coefficients.
        pointlist is list of N points, each [x,y]"""
        # shape is (N, 2)
        pointarray: NDArray[np.float32] = np.array(pointlist, dtype=np.float32)
        # points shape is (N, 1, 2)
        points: NDArray[np.float32] = pointarray.reshape((-1, 1, 2))
        # points: NDArray[np.float32] = pointarray.reshape((-1, 2))
        # undistorted shape is (N, 1, 2)
        undistorted: MatLike = cv2.undistortImagePoints(points, self._mtx, self._dist)
        return undistorted.reshape(-1, 2)

    def analyze(self, req: Request) -> None:
        buffer: Buffer
        with req.rgb() as buffer:
            # microsecond age of frame
            delay_us = req.delay_us()

            # localtime in microseconds
            localtime: int = int(ntcore._now() - delay_us)  # pylint:disable=W0212
            servertime: int = self._network.server_time(localtime)

            img: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)

            img_bgr: NDArray[np.uint8] = img.reshape((self._height, self._width, 3))

            img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
            img_hsv = np.ascontiguousarray(img_hsv)

            img_range = cv2.inRange(img_hsv, self._object_lower, self._object_higher)

            floodfill = img_range.copy()
            mask = np.zeros((self._height + 2, self._width + 2), np.uint8)
            cv2.floodFill(floodfill, mask, [0, 0], [255])

            # floodfill_inv = cv2.bitwise_not(floodfill)
            # img_floodfill = cv2.bitwise_or(img_range, floodfill_inv)
            median = cv2.medianBlur(img_range, 5)

            contours, _ = cv2.findContours(
                median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )

            targets: list[Target] = []
            # list of [x, y]
            points_to_undistort: list[list[int]] = []
            contour_info: list[tuple[MatLike, int, int]] = []

            for contour in contours:
                # https://en.wikipedia.org/wiki/Image_moment
                mmnts: Moments = cv2.moments(contour)

                # reject too small (m00 is in pixels)
                if mmnts["m00"] < 100:
                    continue

                cX: int = int(mmnts["m10"] / mmnts["m00"])
                cY: int = int(mmnts["m01"] / mmnts["m00"])

                points_to_undistort.append([cX, cY])
                contour_info.append((contour, cX, cY))

            if points_to_undistort:
                # Undistort all points at once
                undistorted_points = self.undistort_points(points_to_undistort)

                c: MatLike
                orig_cX: int
                orig_cY: int
                undist_cX: float
                undist_cY: float
                for (c, orig_cX, orig_cY), (undist_cX, undist_cY) in zip(
                    contour_info, undistorted_points
                ):

                    yNormalized: float = (self._height / 2 - undist_cY) / self._mtx[
                        1, 1
                    ]
                    xNormalized: float = (self._width / 2 - undist_cX) / self._mtx[0, 0]

                    initial = np.array([1, 0, 0], dtype=np.float64)
                    final = np.array([1, xNormalized, yNormalized], dtype=np.float64)
                    rotation = Rotation3d(initial=initial, final=final)

                    targets.append(Target(servertime, rotation))
                    self._display.note(img_bgr, c, orig_cX, orig_cY)

                # only send if there's anything to say
                self._targets.send(targets)
                # must flush!  otherwise 100ms update rate.
                self._network.flush()

            fps = req.fps()
            self._display.text(img_bgr, f"FPS {fps:2.0f}", (5, 65))
            self._display.text(img_bgr, f"delay (ms) {delay_us/1000:2.0f}", (5, 105))
            self._display.put(img_bgr)
