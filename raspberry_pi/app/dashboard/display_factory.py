from app.camera.camera_protocol import Camera
from app.camera.size import Size
from app.config.identity import Identity
from app.dashboard.display import Display
from app.dashboard.fake_display import FakeDisplay
from app.dashboard.real_display import RealDisplay


class DisplayFactory:
    @staticmethod
    def get(identity: Identity, cam: Camera) -> Display:
        match identity:
            case (
                Identity.CORAL_LEFT
                | Identity.CORAL_RIGHT
                | Identity.DEV
                | Identity.DEV2
                | Identity.DIST_TEST
                | Identity.FUNNEL
                | Identity.GAME_PIECE
                | Identity.GLOBAL_GAME_PIECE
                | Identity.LEFTAMP
                | Identity.JOELS_TEST
                | Identity.RIGHTAMP
                | Identity.SHOOTER
                | Identity.SWERVE_LEFT
                | Identity.SWERVE_RIGHT
            ):
                size: Size = cam.get_size()
                scale: float = DisplayFactory.get_scale(identity)
                display_width: int = int(scale * size.width)
                display_height = int(scale * size.height)
                return RealDisplay(display_width, display_height)
            case _:
                return FakeDisplay()

    @staticmethod
    def get_scale(identity: Identity) -> float:
        match identity:
            case Identity.DIST_TEST:
                return 1.0  # full size for debugging; slow.
            case Identity.DEV:  # on the camera bot at the moment
                return 0.25  # ok for dashboard
                # scale = 1.0 # full size for debugging; slow.
            case Identity.FUNNEL:
                return 0.5
            case Identity.UNKNOWN:
                return 1.0  # full size for debugging; slow.
            case _:
                return 0.25  # ok for dashboard
