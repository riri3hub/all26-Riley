# pylint: disable=R0903,W0611

from app.camera.config.config_protocol import Config
from app.camera.size import Size
from app.config.identity import Identity
from app.camera.config.config_thrifty_mjpeg import ConfigThriftyMjpeg
from app.camera.config.config_gs_color import ConfigGsColor
from app.camera.config.config_gs_mono import ConfigGsMono


class ConfigFactory:
    """Choose a camera confing based on identity."""

    @staticmethod
    def get(identity: Identity, size: Size) -> Config:
        match identity:
            case Identity.FUNNEL:
                return ConfigThriftyMjpeg(size)
                # return ConfigThriftyYuyv(size)
            case Identity.CLIMB_RIGHT:
                return ConfigThriftyMjpeg(size)
            case Identity.CLIMB_LEFT:
                return ConfigGsColor(size)
            case Identity.SHOOTER:
                return ConfigGsMono(size)
            case _:
                return ConfigGsMono(size)
