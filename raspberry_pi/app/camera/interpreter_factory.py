# pylint: disable=C0301,E0611,E1101,R0903
import numpy as np
from numpy.typing import NDArray

from app.camera.camera_protocol import Camera
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.display import Display
from app.localization.combined_detector import CombinedDetector
from app.localization.target_detector import TargetDetector
from app.localization.tag_detector import TagDetector
from app.network.network_protocol import Network


class InterpreterFactory:
    @staticmethod
    def get(
        identity: Identity, cam: Camera, display: Display, network: Network
    ) -> Interpreter:
        # GREEN TARGET VALUES
        # object_lower = np.array((40, 50, 100))
        # object_higher = np.array((70, 255, 255))
        # get these values from changing the range till the object is highlighted

        # WHITE TARGET VALUES
        object_lower: NDArray[np.int32] = np.array((0, 0, 200))
        object_higher: NDArray[np.int32] = np.array((255, 150, 255))

        match identity:
            case Identity.FUNNEL:
                return TagDetector(identity, cam, display, network)
            case Identity.GAME_PIECE:
                return TargetDetector(
                    cam,
                    display,
                    network,
                    object_lower,
                    object_higher,
                )
            case (
                Identity.RIGHTAMP
                | Identity.LEFTAMP
                | Identity.SHOOTER
                | Identity.GLOBAL_GAME_PIECE
                | Identity.SWERVE_RIGHT
                | Identity.SWERVE_LEFT
                | Identity.DIST_TEST
                | Identity.JOELS_TEST
                | Identity.DEV
            ):
                return TagDetector(identity, cam, display, network)
            case Identity.DEV2 | Identity.CORAL_RIGHT | Identity.CORAL_LEFT:
                return CombinedDetector(
                    identity,
                    cam,
                    display,
                    network,
                    object_lower,
                    object_higher,
                )
            case _:
                return TagDetector(identity, cam, display, network)
