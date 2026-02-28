"""For testing."""

from typing_extensions import override
from app.network.structs import Blip, Target
from app.network.network_protocol import DoubleSender, BlipSender, TargetSender, Network


class FakeDoubleSender(DoubleSender):
    def __init__(self, fake: "FakeNetwork") -> None:
        self._net = fake

    @override
    def send(self, val: float) -> None:
        self._net.doubles.append(val)


class FakeBlipSender(BlipSender):
    def __init__(self, fake: "FakeNetwork") -> None:
        self._net = fake

    @override
    def send(self, val: list[Blip]) -> None:
        self._net.blips.extend(val)


class FakeTargetSender(TargetSender):
    def __init__(self, fake: "FakeNetwork") -> None:
        self._net = fake

    @override
    def send(self, val: list[Target]) -> None:
        self._net.targets.extend(val)


class FakeNetwork(Network):
    def __init__(self) -> None:
        self.doubles: list[float] = []
        self.blips: list[Blip] = []
        self.targets: list[Target] = []

    @override
    def calibrate(self) -> bool:
        return False

    @override
    def flush(self) -> None:
        pass

    @override
    def get_double_sender(self, leaf: str) -> DoubleSender:
        return FakeDoubleSender(self)

    @override
    def get_blip_sender(self) -> BlipSender:
        return FakeBlipSender(self)

    @override
    def get_target_sender(self) -> TargetSender:
        return FakeTargetSender(self)

    @override
    def server_time(self, localtime: int) -> int:
        return 0
