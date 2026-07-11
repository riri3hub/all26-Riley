# pylint: disable=C0301,E0611,E1101,R0903,R0913,R0917
import numpy as np
from numpy.typing import NDArray

from app.camera.camera_protocol import Camera
from app.config.identity import Identity
from app.dashboard.display_protocol import Display
from app.interpreter.interpreter_protocol import Interpreter
from app.interpreter.viewfinder import Viewfinder
from app.localization.apriltags import AprilTags
from app.localization.blobs import Blobs
from app.localization.combined_detector import CombinedDetector
from app.localization.tag_detector import TagDetector
from app.localization.target_detector import TargetDetector
from app.network.network_protocol import Network
from app.util.timestamps import Timestamps

USE_NULL: bool = False


class InterpreterFactory:
    @staticmethod
    def get(
        identity: Identity,
        cam: Camera,
        display1: Display,
        display2: Display,
        network: Network,
        timestamps: Timestamps,
    ) -> Interpreter:
        if USE_NULL:
            # For testing.
            return Viewfinder(display1, display2, network)

        # GREEN TARGET VALUES
        # object_lower = np.array((40, 50, 100))
        # object_higher = np.array((70, 255, 255))
        # get these values from changing the range till the object is highlighted

        # WHITE TARGET VALUES
        object_lower: NDArray[np.int32] = np.array((0, 0, 200))
        object_higher: NDArray[np.int32] = np.array((255, 150, 255))

        match identity:
            case Identity.FUNNEL:
                return TagDetector(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    AprilTags(identity, cam, network),
                )
            case Identity.GAME_PIECE:
                return TargetDetector(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    Blobs(cam, network, object_lower, object_higher),
                )
            case (
                Identity.CAMERA_FRONT
                | Identity.DEV
                | Identity.DIST_TEST
                | Identity.LEFTAMP
                | Identity.CAMERA_BACK
                | Identity.JOELS_TEST
                | Identity.RIGHTAMP
                | Identity.SHOOTER
                | Identity.CLIMB_LEFT
                | Identity.CLIMB_RIGHT
                | Identity.SWERVE_RIGHT
                | Identity.SWERVE_LEFT
            ):
                return TagDetector(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    AprilTags(identity, cam, network),
                )
            case Identity.DEV2:
                return CombinedDetector(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    AprilTags(identity, cam, network),
                    Blobs(cam, network, object_lower, object_higher),
                )
            case _:
                return TagDetector(
                    cam,
                    display1,
                    display2,
                    network,
                    timestamps,
                    AprilTags(identity, cam, network),
                )
