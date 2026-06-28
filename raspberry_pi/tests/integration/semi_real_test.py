import unittest
import time

from threading import Event, Thread
from app.camera.camera_loop import CameraLoop
from app.camera.camera_protocol import Camera
from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.display import Display
from app.dashboard.fake_display import FakeDisplay
from app.interpreter.interpreter_protocol import Interpreter
from app.localization.tag_detector import TagDetector
from app.network.network_protocol import Network
from app.network.real_network import RealNetwork
from app.util.timestamps import Timestamps


class SemiRealTest(unittest.TestCase):
    # don't run this all the time, it takes too long.
    # @unittest.skip
    def test_end_to_end(self) -> None:
        """Use a fake image with a real network, and run
        for awhile, to test the listener part (in Java).
        This works just like main().
        The file it uses shows tag 1 in a position mostly yawed
        to the left.  In 2026 this tag was around 3/4 of the
        way down the field, near the left side, facing back
        towards the baseline, so the correct robot position is
        about a half meter away, facing about 45 degrees to the
        right."""
        done: Event = Event()  # to shut down all threads

        # unknown uses localhost for the server
        identity: Identity = Identity.SIM0
        camera: Camera = FakeCamera("images/tag_and_board.jpg", (1100, 620), -0.1)
        display: Display = FakeDisplay()
        network: Network = RealNetwork(identity, done)
        timestamps = Timestamps(network)
        interpreter: Interpreter = TagDetector(
            identity, camera, display, network, timestamps
        )
        camera_loop: CameraLoop = CameraLoop(camera, interpreter, done)
        thread: Thread = Thread(target=camera_loop.run)
        thread.start()
        time.sleep(60)
        done.set()
        # looper.run will return when done, so wait for that.
        thread.join()
