# pylint: disable=E1101
import unittest

import numpy as np
import cv2

from numpy.typing import NDArray

from app.camera.fake_camera import FakeCamera
from app.dashboard.fake_display import FakeDisplay
from app.localization.target_detector import TargetDetector
from app.network.structs import Target
from app.network.fake_network import FakeNetwork


class TargetDetectorTest(unittest.TestCase):
    KEY = "objectVision/unknown/targets"

    def test_one_note_found(self) -> None:

        network = FakeNetwork()
        # this has an orange blob that matches the
        # HSV range in the note detector
        # the blob is in the lower right quadrant, so the result
        # should be pitch-down yaw-right.
        # ORANGE TARGET
        # camera = FakeCamera("blob.jpg")
        # GREEN PRACTICE TARGET
        # no distortion
        camera = FakeCamera("green_blob.jpg")
        display = FakeDisplay()

        # GREEN TARGET VALUES
        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))
        note_detector = TargetDetector(
            camera, display, network, object_lower, object_higher
        )
        request = camera.capture_request()
        note_detector.analyze(request)

        self.assertEqual(1, len(display.notes))
        self.assertEqual(1, len(display.circles))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertEqual(498, display.circles[0][0])
        self.assertEqual(414, display.circles[0][1])

        rots: list[Target] = network.targets
        self.assertEqual(1, len(rots))
        rot = rots[0].sight
        # NOTE: 0.01 rad resolution is all that can be expected.
        # ~zero
        self.assertAlmostEqual(-0.01, rot.x, 2)
        # pitch down
        self.assertAlmostEqual(0.12, rot.y, 2)
        # yaw right
        self.assertAlmostEqual(-0.10, rot.z, 2)
        rot2d = rot.toRotation2d()
        # right yaw is about 10 deg
        self.assertAlmostEqual(-5.98, rot2d.degrees(), 2)
        q = rot.getQuaternion()
        self.assertAlmostEqual(0, q.X(), 2)
        self.assertAlmostEqual(0.06, q.Y(), 2)
        self.assertAlmostEqual(-0.05, q.Z(), 2)
        self.assertAlmostEqual(1.0, q.W(), 2)

    def test_inverse(self) -> None:
        # 3d model of the green blob
        points = np.array([[[98, 114, 935]]], dtype=np.float32)
        rvec = np.zeros((3, 1), np.float32)
        tvec: NDArray[np.float32] = np.zeros((3, 1), np.float32)

        mtx: NDArray[np.float32] = np.array(
            [
                [935, 0, 400],
                [0, 935, 300],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )
        # undistorted camera duplicates the points above
        dst = np.array([0, 0, 0, 0], dtype=np.float32)
        uv, _ = cv2.projectPoints(points, rvec, tvec, mtx, dst)
        p = uv[0][0]
        self.assertEqual(498, p[0])
        self.assertEqual(414, p[1])

        # now with the distorted camera
        dst = np.array([-7, 0, 0, 0], dtype=np.float32)
        uv, _ = cv2.projectPoints(points, rvec, tvec, mtx, dst)
        p = uv[0][0]
        self.assertAlmostEqual(480.3, p[0], 1)
        self.assertAlmostEqual(393.4, p[1], 1)

    def test_target_undistort(self) -> None:
        # includes a lot of distortion; in this case it's
        # "barrel" to keep the blob in the frame
        camera = FakeCamera("green_blob.jpg", None, -7, 11.875)
        display = FakeDisplay()
        network = FakeNetwork()

        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))
        note_detector = TargetDetector(
            camera, display, network, object_lower, object_higher
        )
        request = camera.capture_request()
        note_detector.analyze(request)

        self.assertEqual(1, len(display.notes))
        self.assertEqual(1, len(display.circles))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        # display shows distorted values
        # within a pixel or so
        self.assertEqual(480, display.circles[0][0])
        self.assertEqual(394, display.circles[0][1])

        # the extracted rotation should be undistorted.

        rots: list[Target] = network.targets
        self.assertEqual(1, len(rots))
        rot = rots[0].sight
        # NOTE: 0.01 rad resolution is all that can be expected.
        # ~zero
        self.assertAlmostEqual(-0.01, rot.x, 2)
        # pitch down
        self.assertAlmostEqual(0.12, rot.y, 2)
        # yaw right
        self.assertAlmostEqual(-0.10, rot.z, 2)
        rot2d = rot.toRotation2d()
        # almost right
        self.assertAlmostEqual(-5.98, rot2d.degrees(), delta=0.02)
        q = rot.getQuaternion()
        self.assertAlmostEqual(0, q.X(), 2)
        self.assertAlmostEqual(0.06, q.Y(), 2)
        self.assertAlmostEqual(-0.05, q.Z(), 2)
        self.assertAlmostEqual(1.0, q.W(), 2)

    def test_zero_notes_found(self) -> None:
        network = FakeNetwork()

        # nothing in this image
        camera = FakeCamera("white_square.jpg")
        display = FakeDisplay()

        # GREEN TARGET VALUES
        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))
        note_detector = TargetDetector(
            camera, display, network, object_lower, object_higher
        )
        request = camera.capture_request()
        note_detector.analyze(request)

        self.assertEqual(0, len(display.notes))
        self.assertEqual(0, len(display.circles))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        ## always publish even if empty

        rots: list[Target] = network.targets

        self.assertEqual(0, len(rots))
