# pylint: disable=E1101,R0903
import cv2
import numpy as np
from numpy.typing import NDArray
from cv2.typing import MatLike
from robotpy_apriltag import AprilTagDetection

# Tag corners for computing homography.
SRC_POINTS = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]])


class DetectorUtil:
    """Methods used by multiple detectors."""

    @staticmethod
    def raw_corners(
        result_item: AprilTagDetection,
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """Return corners from a detection as a tuple.
        The order is: lower left, lower right, upper right, upper left."""
        return result_item.getCorners((0, 0, 0, 0, 0, 0, 0, 0))

    @staticmethod
    def undistorted_corners(
        mtx: NDArray[np.float32],
        dist: NDArray[np.float32],
        corners: tuple[float, float, float, float, float, float, float, float],
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """Return undistorted tag corners.
        undistortPoints is at least 10X faster than undistort on the whole image."""

        # undistortImagePoints takes [u,v] pixel pairs
        # (MatOfPoint2f in c)
        pairs: MatLike = np.reshape(corners, [4, 2])
        # This is just undistortPoints with mtx as the new intrinsic.
        # The default iterates 5 times and often doesn't get there,
        # so we iterate more times.
        pairs = cv2.undistortImagePoints(
            pairs,
            mtx,
            dist,
            None,
            (cv2.TermCriteria_COUNT | cv2.TermCriteria_EPS, 40, 0.01),
        )

        # The estimator wants a flat tuple: [x0, y0, x1, y1, ...];
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

    @staticmethod
    def homography(
        corners: tuple[float, float, float, float, float, float, float, float],
    ):
        """Use OpenCV to compute the homography."""
        dst_points = np.array(
            [
                [corners[0], corners[1]],
                [corners[2], corners[3]],
                [corners[4], corners[5]],
                [corners[6], corners[7]],
            ]
        )
        h, _ = cv2.findHomography(SRC_POINTS, dst_points)
        return (
            h[0, 0],
            h[0, 1],
            h[0, 2],
            h[1, 0],
            h[1, 1],
            h[1, 2],
            h[2, 0],
            h[2, 1],
            h[2, 2],
        )
