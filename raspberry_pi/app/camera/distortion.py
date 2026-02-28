import numpy as np
from numpy.typing import NDArray

from app.camera.model import Model
from app.config.identity import Identity


class Distortion:
    """Camera distortion matrix.

    This should be measured using mrcal.

    We've had good results using the four-parameter model: [k1, k2, p1, p2].

    See https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
    See https://docs.google.com/spreadsheets/d/1x2_58wyVb5e9HJW8WgakgYcOXgPaJe0yTIHew206M-M
    See doc/calibration.md.
    """

    def __init__(self, identity: Identity, model: Model) -> None:
        self._dist: NDArray[np.float32]

        match model:
            case Model.V3_WIDE:
                self._dist = Distortion.__dist_v3()
            case Model.V2:
                self._dist = Distortion.__dist_v2()
            case Model.GS:
                self._dist = Distortion.__dist_gs(identity)
            case _:
                self._dist = np.array([[0, 0, 0, 0]])

        print("\n*** DISTORTION")
        print(self._dist)

    def get(self) -> NDArray[np.float32]:
        return self._dist

    @staticmethod
    def __dist_v2() -> NDArray[np.float32]:
        return np.array([[-0.003, 0.04, 0, 0]])  # type: ignore

    @staticmethod
    def __dist_v3() -> NDArray[np.float32]:
        return np.array([[0.01, -0.0365, 0, 0]])  # type:ignore

    @staticmethod
    def __dist_gs(identity: Identity) -> NDArray[np.float32]:
        """Distortion matrices for the 3.2 mm lens only.  Do not use the 6 mm lens."""
        match identity:
            case Identity.CORAL_LEFT:
                return np.array(
                    [
                        [
                            -0.2883685917,
                            0.08640059653,
                            -0.0003167335742,
                            0.0004112607248,
                        ]
                    ]
                )
            case Identity.CORAL_RIGHT:
                return np.array(
                    [
                        [
                            -0.3057497651,
                            0.1211648432,
                            0.0003595528879,
                            -0.0002945429981,
                        ]
                    ]
                )
            case _:
                print("###########################################")
                print("#                                         #")
                print("#  CAMERA DISTORTION IS NOT CALIBRATED!   #")
                print("#                                         #")
                print("#  YOU MUST CALIBRATE IT!!!!              #")
                print("#                                         #")
                print("###########################################")
                return np.array(
                    [
                        [
                            -0.2883685917,
                            0.08640059653,
                            -0.0003167335742,
                            0.0004112607248,
                        ]
                    ]
                )
