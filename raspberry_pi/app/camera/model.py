# pylint: disable=E0401

from enum import Enum, unique
from typing import Any
from app.config.identity import Identity


@unique
class Model(Enum):
    V3_WIDE = "imx708_wide"
    V2 = "imx219"
    GS = "imx296"
    THRIFTY = "SPCA2630 PC Camera: PC Camera"
    UNKNOWN = "unknown"

    @classmethod
    def _missing_(cls, value: object) -> Any:
        return Identity.UNKNOWN

    @staticmethod
    def get(camera_properties: dict[str, str]) -> "Model":  # type:ignore
        """Use the properties to avoid import errors in tests."""
        model_str: str = camera_properties["Model"]  # type:ignore
        print(f"\n*** Camera model string: {model_str}")
        model: Model = Model(model_str)
        print(f"\n*** Camera model: {model.name}")
        return model
