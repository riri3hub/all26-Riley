import unittest
from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.localization.tag_detector import TagDetector
from app.network.fake_network import FakeNetwork


class TagDetectorTest(unittest.TestCase):

    # A series of tests of progressively smaller targets.
    # Some of these are sensitive to decimation.

    def test_big_sharp(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("big_sharp.png")
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(1, len(display.tags))
        self.assertEqual(1, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertAlmostEqual(703, display.tags[0].getCenter().x, 0)
        self.assertAlmostEqual(522, display.tags[0].getCenter().y, 0)
        self.assertAlmostEqual(0.000, display.poses[0].x, 3)
        self.assertAlmostEqual(-0.006, display.poses[0].y, 3)
        self.assertAlmostEqual(0.264, display.poses[0].z, 3)

    def test_scale1(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("scale1.png")
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(1, len(display.tags))
        self.assertEqual(1, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertAlmostEqual(703, display.tags[0].getCenter().x, -1)
        self.assertAlmostEqual(532, display.tags[0].getCenter().y, 0)
        self.assertAlmostEqual(0.000, display.poses[0].x, 3)
        self.assertAlmostEqual(-0.007, display.poses[0].y, 3)
        self.assertAlmostEqual(0.528, display.poses[0].z, 3)

    def test_scale2(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("scale2.png")
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(1, len(display.tags))
        self.assertEqual(1, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertAlmostEqual(703, display.tags[0].getCenter().x, 0)
        self.assertAlmostEqual(537, display.tags[0].getCenter().y, 0)
        self.assertAlmostEqual(-0.001, display.poses[0].x, 3)
        self.assertAlmostEqual(-0.008, display.poses[0].y, 3)
        self.assertAlmostEqual(1.055, display.poses[0].z, 3)

    def test_scale3(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("scale3.pnm")
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(1, len(display.tags))
        self.assertEqual(1, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertAlmostEqual(703, display.tags[0].getCenter().x, 0)
        self.assertAlmostEqual(541, display.tags[0].getCenter().y, 0)
        self.assertAlmostEqual(-0.003, display.poses[0].x, 3)
        self.assertAlmostEqual(-0.007, display.poses[0].y, 3)
        # quad decimate of 4 makes a 0.1% difference here
        self.assertAlmostEqual(2.119, display.poses[0].z, 3)

    def test_scale4(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("scale4.pnm")
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(1, len(display.tags))
        self.assertEqual(1, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertAlmostEqual(703, display.tags[0].getCenter().x, 0)
        self.assertAlmostEqual(543, display.tags[0].getCenter().y, 0)
        self.assertAlmostEqual(-0.005, display.poses[0].x, 3)
        self.assertAlmostEqual(-0.007, display.poses[0].y, 3)
        # quad decimate of 4 makes a 1% difference here.
        self.assertAlmostEqual(4.242, display.poses[0].z, 3)

    def test_scale5(self) -> None:
        # it's kind of amazing that this one works
        # this does not work when quadDecimate is set to 4.
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("scale5.pnm")
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(0, len(display.tags))

        # turn quadDecimate back to 1, and this should work again.
        # self.assertEqual(1, len(display.tags))
        # self.assertEqual(1, len(display.poses))
        # self.assertEqual(2, len(display.msgs))
        # self.assertEqual(2, len(display.locs))
        # self.assertEqual(1, display.frame_count)

        # self.assertAlmostEqual(703, display.tags[0].getCenter().x, 0)
        # self.assertAlmostEqual(543, display.tags[0].getCenter().y, 0)
        # self.assertAlmostEqual(0, display.poses[0].x, 1)
        # self.assertAlmostEqual(0, display.poses[0].y, 1)
        # self.assertAlmostEqual(8.512, display.poses[0].z, 3)

    def test_scale6(self) -> None:
        # now finally too small
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("scale6.pnm")
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(0, len(display.tags))
        self.assertEqual(0, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

    def test_one_tag_found(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("tag_and_board.jpg", (1100, 620))
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(1, len(display.tags))
        self.assertEqual(1, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertAlmostEqual(282, display.tags[0].getCenter().x, 0)
        self.assertAlmostEqual(349, display.tags[0].getCenter().y, 0)
        self.assertAlmostEqual(-0.186, display.poses[0].x, 3)
        self.assertAlmostEqual(0.027, display.poses[0].y, 3)
        self.assertAlmostEqual(0.642, display.poses[0].z, 3)

    def test_zero_tags_found(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        # nothing in this image
        camera = FakeCamera("white_square.jpg")
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(0, len(display.tags))
        self.assertEqual(0, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

    def test_distortion(self) -> None:
        """How much distortion can there be in the image?"""
        identity = Identity.UNKNOWN
        network = FakeNetwork()

        # no distortion, like above
        camera = FakeCamera("tag_and_board.jpg", (1100, 620), 0, 0)
        display = FakeDisplay()
        TagDetector(identity, camera, display, network).analyze(
            camera.capture_request()
        )
        self.assertEqual(1, len(display.tags))

        # this is about the most possible
        camera = FakeCamera("tag_and_board.jpg", (1100, 620), -5, 5)
        display = FakeDisplay()
        TagDetector(identity, camera, display, network).analyze(
            camera.capture_request()
        )
        self.assertEqual(1, len(display.tags))

        # This is too much distortion, so detection fails.
        # Note, this is a truly enormous amount of distortion.
        camera = FakeCamera("tag_and_board.jpg", (1100, 620), -50, 50)
        display = FakeDisplay()
        TagDetector(identity, camera, display, network).analyze(
            camera.capture_request()
        )
        self.assertEqual(0, len(display.tags))
