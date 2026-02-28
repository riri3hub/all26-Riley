"""This is the coprocessor main loop.

Each task is run by its own Looper, in its own thread.

You can't run this from the command line.  To run the app,
use the script called "runapp.py" in the raspberry_pi directory
(one level above this one).
"""

# pylint: disable=R0914

from queue import Queue
from threading import Event, Thread
import ntcore
from app.sync.sync_loop import SyncLoop
from app.test.drift_loop import DriftLoop


def main() -> None:
    print("*** main")

    done = Event()
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("sync_client")
    inst.setServer("10.1.0.2")

    queue: Queue = Queue()

    syncloop = SyncLoop(inst, queue, done)
    driftloop = DriftLoop(inst, queue, done)

    Thread(target=syncloop.run).start()
    Thread(target=driftloop.run).start()

    done.wait()
