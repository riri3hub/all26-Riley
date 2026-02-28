import unittest

from app.network.fake_network import FakeNetwork


class FakeNetworkTest(unittest.TestCase):
    def test_double(self) -> None:
        network = FakeNetwork()
        sender = network.get_double_sender("foo")
        sender.send(1)
        self.assertEqual(1, network.doubles[0])
