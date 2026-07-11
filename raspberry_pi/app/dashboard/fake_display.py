from cv2.typing import MatLike
from typing_extensions import override
from app.dashboard.display_protocol import Display


class FakeDisplay(Display):
    """A display for unit tests."""

    def __init__(self) -> None:
        print("\n*** Display: FakeDisplay")
        self.frame_count = 0

    @override
    def put(self, img: MatLike) -> None:
        self.frame_count += 1
