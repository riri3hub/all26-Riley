# pylint: disable=E1101
import unittest

import numpy as np
import cv2

from numpy.typing import NDArray

from app.camera.fake_camera import FakeCamera
from app.dashboard.fake_display import FakeDisplay
from app.localization.blobs import Blobs
from app.localization.target_detector import TargetDetector
from app.network.structs import Target
from app.network.fake_network import FakeNetwork
from app.util.timestamps import Timestamps


class TargetDetectorTest(unittest.TestCase):
    KEY = "objectVision/unknown/targets"

    def test_one_note_found(self) -> None:

        network = FakeNetwork()
        timestamps = Timestamps(network)
        # this has a blob that matches the
        # HSV range in the note detector
        # the blob is in the lower right quadrant, so the result
        # should be pitch-down yaw-right.
        # ORANGE TARGET
        # camera = FakeCamera("images/blob.jpg")
        # GREEN PRACTICE TARGET
        # no distortion
        camera = FakeCamera("images/green_blob.jpg")
        display1 = FakeDisplay()
        display2 = FakeDisplay()

        # GREEN TARGET VALUES
        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))
        note_detector = TargetDetector(
            camera,
            display1,
            display2,
            network,
            timestamps,
            Blobs(camera, network, object_lower, object_higher),
        )
        request = camera.capture_request()
        note_detector.interpret(request)

        self.assertEqual(1, display1.frame_count)

        rots: list[Target] = network.targets
        self.assertEqual(1, len(rots))
        rot = rots[0].sight
        # ~zero
        self.assertAlmostEqual(-0.006, rot.x, delta=0.001)
        # pitch down
        self.assertAlmostEqual(0.121, rot.y, delta=0.001)
        # yaw right
        self.assertAlmostEqual(-0.104, rot.z, delta=0.001)
        rot2d = rot.toRotation2d()
        # right yaw is about 10 deg
        self.assertAlmostEqual(-5.983, rot2d.degrees(), delta=0.001)
        q = rot.getQuaternion()
        self.assertAlmostEqual(0, q.X(), delta=0.001)
        self.assertAlmostEqual(0.060, q.Y(), delta=0.001)
        self.assertAlmostEqual(-0.052, q.Z(), delta=0.001)
        self.assertAlmostEqual(0.997, q.W(), delta=0.001)

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
        # "barrel" to keep the blob in the frame
        camera = FakeCamera("images/green_blob.jpg", None, -0.1)
        display1 = FakeDisplay()
        display2 = FakeDisplay()
        network = FakeNetwork()
        timestamps = Timestamps(network)

        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))
        note_detector = TargetDetector(
            camera,
            display1,
            display2,
            network,
            timestamps,
            Blobs(camera, network, object_lower, object_higher),
        )
        request = camera.capture_request()
        note_detector.interpret(request)

        self.assertEqual(1, display1.frame_count)

        # the extracted rotation should be undistorted.
        rots: list[Target] = network.targets
        self.assertEqual(1, len(rots))
        rot = rots[0].sight
        # NOTE: 0.01 rad resolution is all that can be expected.
        # ~zero
        self.assertAlmostEqual(-0.006, rot.x, delta=0.001)
        # pitch down
        self.assertAlmostEqual(0.121, rot.y, delta=0.001)
        # yaw right
        self.assertAlmostEqual(-0.104, rot.z, delta=0.001)
        rot2d = rot.toRotation2d()
        # almost right
        self.assertAlmostEqual(-5.938, rot2d.degrees(), delta=0.001)
        q = rot.getQuaternion()
        self.assertAlmostEqual(0, q.X(), delta=0.001)
        self.assertAlmostEqual(0.06, q.Y(), delta=0.001)
        self.assertAlmostEqual(-0.052, q.Z(), delta=0.001)
        self.assertAlmostEqual(0.997, q.W(), delta=0.001)

    def test_zero_notes_found(self) -> None:
        network = FakeNetwork()
        timestamps = Timestamps(network)

        # nothing in this image
        camera = FakeCamera("images/white_square.jpg")
        display1 = FakeDisplay()
        display2 = FakeDisplay()

        # GREEN TARGET VALUES
        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))
        note_detector = TargetDetector(
            camera,
            display1,
            display2,
            network,
            timestamps,
            Blobs(camera, network, object_lower, object_higher),
        )
        request = camera.capture_request()
        note_detector.interpret(request)

        self.assertEqual(1, display1.frame_count)

        ## always publish even if empty

        rots: list[Target] = network.targets

        self.assertEqual(0, len(rots))
