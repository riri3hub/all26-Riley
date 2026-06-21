# pylint: disable=R0903

from typing import Any, cast
from app.camera.config.config_protocol import Config
from app.util.timer import Timer

# For when the camera doesn't report the exposure time,
# and there's no configured exposure time either.
DEFAULT_EXPOSURE_US: int = 1000


class Delay:
    """Estimate the age of a frame based on config and metadata."""

    def __init__(self, config: Config) -> None:
        self._extra_us = cast(int, config.extra_delay_ms() * 1000)
        """Calibrated extra delay in microseconds, using "camera_delay" app."""
        self._fallback_exposure_us = self._get_fallback_exposure_us(config)
        """Fallback exposure duration in microseconds."""

    def delay_us(self, metadata: dict[str, Any]) -> int:
        """Delay from capture to the current instant,
        in microseconds."""

        # Time of first row received, in microseconds in the Timer timebase.
        # This is roughly the "readout timestamp".
        sensor_timestamp_us = cast(int, metadata["SensorTimestamp"]) // 1000

        # Delay due to exposure duration.
        half_exposure_us = self._get_half_exposure_us(metadata)

        # The actual capture is a little bit earlier
        exposure_timestamp_us = sensor_timestamp_us - self._extra_us - half_exposure_us

        # The delay is the difference between the exposure time and the current instant.
        now_us: int = Timer.time_ns() // 1000
        return now_us - exposure_timestamp_us

    def _get_half_exposure_us(self, metadata: dict[str, Any]) -> int:
        """The exposure itself takes time, producing a blur on the sensor.
        Assume the target is found in the middle of the blur, so the
        correct time is the middle of the exposure duration."""

        # Divide by 2 to get the "middle" of the exposure duration.
        return self._get_exposure_us(metadata) // 2

    def _get_exposure_us(self, metadata: dict[str, Any]) -> int:
        """Exposure duration from metadata or fallback."""
        if "ExposureTime" in metadata:
            # Sometimes exposure time is available in metadata.
            return cast(int, metadata["ExposureTime"])
        # Sometimes exposure time configured as a constant.
        return self._fallback_exposure_us

    @staticmethod
    def _get_fallback_exposure_us(config: Config) -> int:
        """Obtain the fallback exposure time, in microseconds,
        from config, or use the default."""
        controls = config.controls()
        if controls.get("AeEnable", True):
            # AE is on, or missing, so just guess
            return DEFAULT_EXPOSURE_US
        # Use the configured fixed exposure time
        return controls.get("ExposureTime", DEFAULT_EXPOSURE_US)
