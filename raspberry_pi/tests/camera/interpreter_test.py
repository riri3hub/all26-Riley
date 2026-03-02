import unittest

from app.camera.camera_protocol import Camera
from app.camera.fake_camera import FakeCamera

from app.camera.interpreter_factory import InterpreterFactory
from app.config.identity import Identity
from app.dashboard.display import Display
from app.dashboard.fake_display import FakeDisplay
from app.network.fake_network import FakeNetwork
from app.network.network_protocol import Network


class IntrinsicTest(unittest.TestCase):
    def test_identity(self) -> None:
        identity: Identity = Identity.DEV
        cam: Camera = FakeCamera("tag_and_board.jpg", (5504, 3096), -0.1, 0.1)
        display: Display = FakeDisplay()
        network: Network = FakeNetwork()
        InterpreterFactory.get(identity, cam, display, network)
