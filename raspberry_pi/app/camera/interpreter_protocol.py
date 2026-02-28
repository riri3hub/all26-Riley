"""Interface spec for image interpreters."""

# pylint: disable=R0903

from typing import Protocol
from app.camera.camera_protocol import Request


class Interpreter(Protocol):
    def analyze(self, req: Request) -> None: ...
