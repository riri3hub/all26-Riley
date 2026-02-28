# pylint: disable=E1101,R0903,R1732

from pathlib import Path
from typing import Optional
import cv2
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray
from typing_extensions import override
from app.camera.camera_protocol import Camera
from app.camera.fake_request import FakeRequest
from app.camera.size import Size
from app.util.timer import Timer


class FakeCamera(Camera):
    """A camera for desktop testing."""

    def __init__(
        self,
        filename: str,
        size: Optional[tuple[int, int]] = None,
        k1: float = 0.0,
        inv_k1: float = 0.0,
    ) -> None:
        """
        size: if no size is supplied, the native size is used.
        k1: quadratic distortion term for undistortion
        inv_k1: inverse distortion, used to distort the image
        note: if k1 magnitude is more than about 7, undistort barfs.
        """
        p = Path(__file__).with_name(filename)
        pathstr: str = str(p)
        file = cv2.imread(pathstr)
        if file is None:
            raise ValueError("no file")
        self.img: MatLike = file
        if size is not None:
            self.img = cv2.resize(self.img, size)
        self.h: int = self.img.shape[0]
        self.w: int = self.img.shape[1]
        self.c: int = self.img.shape[2]
        self.frame_time = Timer.time_ns()
        self.k1 = k1
        # use undistort to distort the image, using the inverse
        mtx: NDArray[np.float32] = self.get_intrinsic()

        dist = np.array([inv_k1, 0, 0, 0])

        self.img = cv2.undistort(self.img, mtx, dist)
        # uncomment to see the distorted thing
        cv2.imwrite("blarg.jpg", self.img)

    @override
    def capture_request(self) -> FakeRequest:
        capture_start: int = Timer.time_ns()
        total_time_ms = (capture_start - self.frame_time) / 1000000
        self.frame_time = capture_start
        fps = 1000 / total_time_ms
        return FakeRequest(self.img, fps)

    @override
    def stop(self) -> None:
        pass

    @override
    def get_size(self) -> Size:
        return Size(
            sensor_width=self.w,
            sensor_height=self.h,
            width=self.w,
            height=self.h,
        )

    @override
    def get_intrinsic(self) -> NDArray[np.float32]:
        # convenient for experiments related to the GS camera
        return np.array(
            [
                [935, 0, self.w // 2],
                [0, 935, self.h // 2],
                [0, 0, 1],
            ]
        )

    @override
    def get_dist(self) -> NDArray[np.float32]:
        k1 = self.k1  # radial quadratic term
        k2 = 0  # radial quartic term
        p1 = 0  # tangential
        p2 = 0  # tangential
        return np.array([k1, k2, p1, p2])
