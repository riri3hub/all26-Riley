# pylint: disable=R0903

import numpy as np
from numpy.typing import NDArray

from app.camera.model import Model
from app.config.identity import Identity


class Intrinsic:
    """Camera intrinsic matrix.

    This should be measured using mrcal.

    See doc/calibration.md.
    """

    def __init__(self, identity: Identity, model: Model) -> None:
        self._mtx: NDArray[np.float32]
        match model:
            case Model.V3_WIDE:
                self._mtx = Intrinsic.__mtx_v3()
            case Model.V2:
                self._mtx = Intrinsic.__mtx_v2()
            case Model.GS:
                self._mtx = Intrinsic.__mtx_gs(identity)
            case _:
                self._mtx = np.array(
                    [
                        [100, 0, 50],
                        [0, 100, 50],
                        [0, 0, 1],
                    ]
                )

        print("\n*** INTRINSIC")
        print(self._mtx)

    def get(self) -> NDArray[np.float32]:
        return self._mtx

    @staticmethod
    def __mtx_v2() -> NDArray[np.float32]:
        return np.array(
            [
                [660, 0, 426],
                [0, 660, 303],
                [0, 0, 1],
            ]
        )

    @staticmethod
    def __mtx_v3() -> NDArray[np.float32]:
        return np.array(
            [
                [498, 0, 584],
                [0, 498, 316],
                [0, 0, 1],
            ]
        )

    @staticmethod
    def __mtx_gs(identity: Identity) -> NDArray[np.float32]:
        """Intrinsic matrices for 3.2 mm lens.  Do not use the 6 mm lens."""
        match identity:
            case Identity.DEV:
                return np.array(
                    [
                        [944.3507484, 0.0, 693.7105365],
                        [0.0, 943.8611003, 498.1103206],
                        [0, 0, 1],
                    ]
                )
            case Identity.CORAL_LEFT:
                return np.array(
                    [
                        [935.4403554, 0.0, 676.3779953],
                        [0.0, 934.6111779, 537.0691437],
                        [0, 0, 1],
                    ]
                )
            case Identity.CORAL_RIGHT:
                return np.array(
                    [
                        [938.0364397, 0.0, 674.7133631],
                        [0.0, 937.4685798, 548.7346201],
                        [0, 0, 1],
                    ]
                )
            case _:
                # Default GS model intrinsic matrix
                print("##########################################")
                print("#                                        #")
                print("#  CAMERA INTRINSIC IS NOT CALIBRATED!   #")
                print("#                                        #")
                print("#  YOU MUST CALIBRATE IT!!!!             #")
                print("#                                        #")
                print("##########################################")

                return np.array(
                    [
                        [944.3507484, 0.0, 693.7105365],
                        [0.0, 943.8611003, 498.1103206],
                        [0, 0, 1],
                    ]
                )
