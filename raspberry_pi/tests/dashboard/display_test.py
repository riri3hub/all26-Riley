import unittest
import numpy as np
from numpy.typing import NDArray

from app.dashboard.fake_display import FakeDisplay


class DisplayTest(unittest.TestCase):
    def test_display(self) -> None:
        display = FakeDisplay()
        img: NDArray[np.uint8] = np.zeros([100, 100], dtype=np.uint8)
        display.text(img, "hi", (0, 0))
