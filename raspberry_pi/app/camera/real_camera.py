"""This is a wrapper for Picamera2.

It handles configuration of each camera according to the Pi identity.

For more on the Picamera2 library, see the manual:

https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

and the source:

https://github.com/raspberrypi/picamera2/
"""

# pylint: disable=E0401

from pprint import pprint
from typing import Any

import numpy as np
from numpy.typing import NDArray
from picamera2 import CompletedRequest, Picamera2, libcamera  # type: ignore
from typing_extensions import override

from app.camera.camera_protocol import Camera, Request
from app.camera.distortion import Distortion
from app.camera.intrinsic import Intrinsic
from app.camera.model import Model
from app.camera.real_request import RealRequest
from app.camera.shutter import Shutter
from app.camera.size import Size
from app.config.identity import Identity
from app.util.timer import Timer


class RealCamera(Camera):
    def __init__(self, identity: Identity) -> None:
        self._cam: Picamera2 = Picamera2()  # type: ignore

        print("\n*** SENSOR MODES AVAILABLE")
        pprint(self._cam.sensor_modes)  # type:ignore

        print("\n*** CAMERA CONTROLS")
        pprint(self._cam.camera_controls)  # type:ignore

        model: Model = Model.get(self._cam.camera_properties)  # type: ignore

        self._mtx: Intrinsic = Intrinsic(identity, model)
        self._dist: Distortion = Distortion(identity, model)

        self._rolling = Shutter(model).rolling()
        self._size: Size = Size.from_model(model)

        self._camera_config: dict[str, Any] = RealCamera.__get_config(  # type: ignore
            identity, self._cam, self._size  # type: ignore
        )

        print("\n*** REQUESTED CONFIG")
        print(self._camera_config)

        # optimal alignment makes the ISP a little faster
        self._cam.align_configuration(self._camera_config, optimal=True)  # type:ignore
        print("\n*** ALIGNED CONFIG")
        print(self._camera_config)

        self._cam.configure(self._camera_config)  # type:ignore

        self._fail_mismatched_size()

        self._cam.start()  # type:ignore
        self._frame_time = Timer.time_ns()

    @override
    def capture_request(self) -> Request:
        capture_start: int = Timer.time_ns()
        req: CompletedRequest = self._cam.capture_request()  # type:ignore
        total_time_ms = (capture_start - self._frame_time) / 1000000
        self._frame_time = capture_start
        fps = 1000 / total_time_ms
        return RealRequest(req, fps, self._rolling)  # type: ignore

    @override
    def stop(self) -> None:
        self._cam.stop()  # type: ignore
        print("Camera stop")

    @override
    def get_size(self) -> Size:
        return self._size

    @override
    def get_intrinsic(self) -> NDArray[np.float32]:
        return self._mtx.get()

    @override
    def get_dist(self) -> NDArray[np.float32]:
        return self._dist.get()

    @staticmethod
    def __get_config(
        identity: Identity, cam: Picamera2, size: Size  # type: ignore
    ) -> dict[str, Any]:
        """Consult https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf"""
        camera_config: dict[str, Any] = cam.create_still_configuration(  # type:ignore
            # more buffers seem to make the pipeline a little smoother
            buffer_count=5,
            queue=True,
            sensor={
                "output_size": (size.sensor_width, size.sensor_height),
                "bit_depth": 10,
            },
            main={"format": "RGB888", "size": (size.width, size.height)},
            lores={"format": "YUV420", "size": (size.width, size.height)},
            raw=None,
            controls={
                # ANALOGUE GAIN
                # To minimize blur, set this as high as possible.
                # TODO: what is the limit here?
                "AnalogueGain": 8,
                #
                # AUTO EXPOSURE
                # Must be true for outside or in bright sun.
                "AeEnable": True,
                # "AeEnable": False,
                #
                # AUTO WHITE BALANCE
                # Screws up color sensing.
                "AwbEnable": False,
                #
                # EXPOSURE TIME (microseconds)
                # Minimizes blur.  Requires pretty good light.
                "ExposureTime": 500,
                # Works in less light, slightly more blur.
                # "ExposureTime": 2000,
                #
                # COLOUR GAINS
                # The first argument is the red gain, second is blue gain.
                # values are from testing in the new gym lighting(1.2,2.2)
                # "ColourGains": (1.2,2.0),
                #
                # FRAME DURATION LIMITS
                # limit auto exposure: go as fast as possible but no slower than 30fps
                # without a duration limit, we slow down in the dark, which is fine
                # "FrameDurationLimits": (500, 33333),  # 41 fps
                #
                # NOISE REDUCTION MODE
                # noise reduction takes A LOT of time (100 ms per frame!), don't need it.
                # See libcamera.controls.draft.NoiseReductionModeEnum.Off,
                "NoiseReductionMode": 0,
            },
        )

        # Flip for upside-down cameras.
        # see libcamera/src/libcamera/transform.cpp
        if identity in (
            Identity.FLIPPED,
            Identity.FUNNEL,
            Identity.SWERVE_LEFT,
            Identity.SWERVE_RIGHT,
        ):
            camera_config["transform"] = libcamera.Transform(  # type: ignore
                rotation=0, hflip=True, vflip=True, transpose=False
            )

        return camera_config  # type: ignore

    def _fail_mismatched_size(self):
        """Configured size and actual size must match."""
        if (
            self._camera_config["sensor"]["output_size"]
            != self._cam.camera_config["sensor"]["output_size"]  # type:ignore
        ):
            raise ValueError(
                "Desired sensor size must match selected sensor size.",
                self._camera_config["sensor"]["output_size"],
                self._cam.camera_config["sensor"]["output_size"],  # type:ignore
            )
