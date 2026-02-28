"""This is the coprocessor main loop.

Each task is run by its own Looper, in its own thread.

You can't run this from the command line.  To run the app,
use the script called "runapp.py" in the raspberry_pi directory
(one level above this one).
"""

# pylint: disable=R0914

from threading import Event, Thread

from app.camera.camera_factory import CameraFactory
from app.camera.camera_protocol import Camera
from app.camera.interpreter_factory import InterpreterFactory
from app.camera.interpreter_protocol import Interpreter
from app.camera.camera_loop import CameraLoop
from app.config.identity import Identity
from app.dashboard.display import Display
from app.dashboard.display_factory import DisplayFactory
from app.network.network_protocol import Network
from app.network.real_network import RealNetwork


def main() -> None:
    identity: Identity = Identity.get()
    done: Event = Event() # to shut down all threads
    try:
        camera: Camera = CameraFactory.get(identity)
        display: Display = DisplayFactory.get(identity, camera)
        network: Network = RealNetwork(identity, done)
        interpreter: Interpreter = InterpreterFactory.get(
            identity, camera, display, network
        )
        camera_loop: CameraLoop = CameraLoop(camera, interpreter, done)
        Thread(target=camera_loop.run).start()
        done.wait()

    finally:
        done.set()  # exit all threads cleanly
