import unittest

from threading import Event

import ntcore

from app.config.identity import Identity
from app.network.real_network import RealNetwork


class RealNetworkTest(unittest.TestCase):
    def test_send(self) -> None:
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getDoubleTopic("pi/unknown/foo").subscribe(0.0)

        network = RealNetwork(Identity.UNKNOWN, Event())
        sender = network.get_double_sender("foo")
        sender.send(1.0)
        self.assertEqual(1.0, sub.get())
