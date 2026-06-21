from typing import Protocol

from cv2.typing import MatLike
from robotpy_apriltag import AprilTagDetection
from wpimath.geometry import Transform3d


class Display(Protocol):
    """Interface spec for dashboard video."""

    def tag(
        self, image: MatLike, tag: AprilTagDetection, pose: Transform3d
    ) -> None: ...
    def note(self, image: MatLike, contour: MatLike, c_x: int, c_y: int) -> None: ...
    def text(self, image: MatLike, msg: str, loc: tuple[int, int]) -> None: ...
    def put(self, img: MatLike) -> None: ...
