# pylint: disable=E1101,R0903

import cv2
from cv2.typing import MatLike
from robotpy_apriltag import AprilTagDetection
from wpimath.geometry import Transform3d


FONT = cv2.FONT_HERSHEY_SIMPLEX
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)


class DisplayUtil:
    """Methods for manipulating images for display."""

    @staticmethod
    def note(image: MatLike, contour: MatLike, c_x: int, c_y: int) -> None:
        """Draw the contour with a circle at the center (c_x, c_y)."""
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 3)
        cv2.circle(image, (c_x, c_y), 7, (0, 0, 0), -1)

    @staticmethod
    def text(image: MatLike, msg: str, loc: tuple[int, int]) -> None:
        """Write msg at loc in img, in WPI coords (x-forward), not camera coordinates (z-forward)"""
        cv2.putText(image, msg, loc, FONT, 3, BLACK, 10)  # black background
        cv2.putText(image, msg, loc, FONT, 3, WHITE, 5)  # white text

    @staticmethod
    def tag(image: MatLike, tag: AprilTagDetection, pose: Transform3d) -> None:
        """Draw the tag outline, and annotate its pose."""
        for i in range(4):
            j = (i + 1) % 4
            point1 = (int(tag.getCorner(i).x), int(tag.getCorner(i).y))
            point2 = (int(tag.getCorner(j).x), int(tag.getCorner(j).y))
            cv2.line(image, point1, point2, WHITE, 2)
        (c_x, c_y) = (int(tag.getCenter().x), int(tag.getCenter().y))
        cv2.circle(image, (c_x, c_y), 10, WHITE, -1)
        t = pose.translation()
        DisplayUtil.text(image, f"X: {t.z:6.3f}", (c_x, c_y))
        DisplayUtil.text(image, f"Y: {-t.x:6.3f}", (c_x, c_y + 80))
        DisplayUtil.text(image, f"Z: {-t.y:6.3f}", (c_x, c_y + 160))
