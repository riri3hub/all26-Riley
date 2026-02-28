from app.camera.model import Model


class Shutter:
    """What kind of shutter does this model have?"""

    def __init__(self, model: Model) -> None:
        match model:
            case Model.GS:
                self._rolling = False
            case _:
                self._rolling = True

    def rolling(self) -> bool:
        return self._rolling
