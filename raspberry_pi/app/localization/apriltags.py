from typing import Any, override
from cv2.typing import MatLike
import numpy as np
from numpy.typing import NDArray

from robotpy_apriltag import AprilTagDetection, AprilTagDetector, AprilTagPoseEstimator

from wpimath.geometry._geometry import Transform3d

from app.camera.camera_protocol import Camera
from app.config.identity import Identity
from app.dashboard.display_util import DisplayUtil
from app.localization.analysis_protocol import MonoAnalysis
from app.localization.detector_util import DetectorUtil
from app.network.network_protocol import Network
from app.network.structs import Blip, BlipWithCorners


class AprilTags(MonoAnalysis):
    """A wrapper for the AprilTag detector."""

    def __init__(self, identity: Identity, cam: Camera, network: Network) -> None:
        """Finds Apriltags.

        :identity: chooses tag size
        :cam: camera implementation
        :display1: shows annotated image
        :display2: optionally shows undistorted image
        :timestamps: timing source.
        """
        print("\n*** MonoAnalysis: AprilTags")
        self._mtx: NDArray[np.float32] = cam.get_intrinsic()
        self._dist: NDArray[np.float32] = cam.get_dist()
        self._blips = network.get_blip_sender()
        self._blips_with_corners = network.get_blip_with_corners_sender()
        self._at_detector = AprilTags.detector()
        if identity == Identity.DIST_TEST:
            # the distortion rig uses a 33 mm, 20% scale, tag.
            tag_size = 0.033
        elif identity == Identity.FUNNEL:
            # for rotation testing rig
            tag_size = 0.059
        else:
            # normal tag size is 6.5 inches
            tag_size = 0.1651
        print("\n*** tag size:", tag_size)
        self._estimator = AprilTags.estimator(tag_size, self._mtx)

    @override
    def analyze_mono(
        self,
        img: MatLike,
        img_display: MatLike,
        servertime: int,
    ) -> None:
        """Detect from img, show on img_display, undistort, extract pose,
        and send pose and (raw) corners over the network.

        :img: 8-bit mono, distorted, for detection.
        :img_display: distorted, for display, annotated with tag positions
        :servertime: drift-corrected server-time microsecond image timestamp"""

        result: list[AprilTagDetection] = self._at_detector.detect(img.data)

        blips: list[Blip] = []
        blips_with_corners: list[BlipWithCorners] = []
        tag: AprilTagDetection
        for tag in result:
            if tag.getHamming() > 0:
                continue

            # Extract raw (x,y) corners from the tag.
            raw_corners: tuple[
                float, float, float, float, float, float, float, float
            ] = DetectorUtil.raw_corners(tag)

            # Undistort the corners.
            undistorted_corners: tuple[
                float, float, float, float, float, float, float, float
            ] = DetectorUtil.undistorted_corners(self._mtx, self._dist, raw_corners)

            # Redo the homography using the undistorted corners.
            homography: tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any] = (
                DetectorUtil.homography(undistorted_corners)
            )

            # Estimate the pose.
            pose: Transform3d = self._estimator.estimate(
                homography, undistorted_corners
            )
            blips.append(Blip(servertime, tag.getId(), pose))
            blips_with_corners.append(
                BlipWithCorners.make(servertime, tag.getId(), raw_corners, pose)
            )
            DisplayUtil.tag(img_display, tag, pose)

        self._blips.send(blips)
        self._blips_with_corners.send(blips_with_corners)

    @staticmethod
    def detector() -> AprilTagDetector:
        at_detector = AprilTagDetector()
        config = at_detector.Config()
        # some of the detection steps can be done in parallel; this
        # should be the same as the number of cores on the machine,
        # which for the Raspberry Pi 5 is four.
        config.numThreads = 4
        # sharpening kernel is [0,-1,0;-1,4,-1;0,-1,0], see apriltag.c
        # this makes adjacent pixels more different.
        config.decodeSharpening = 0.0
        # this doesn't make much difference in FPS
        # config.decodeSharpening = 0.25

        # do not decimate: improves far-away (small) detections
        # config.quadDecimate = 1.0
        # this makes a huge difference in FPS
        # 1.0: 13 FPS
        # 2.0: 40 FPS
        # 4.0: 60 FPS
        # config.quadDecimate = 4.0
        config.quadDecimate = 2.0
        config.refineEdges = True

        # stddev of the blur kernel in pixels: seems to help with small tags
        config.quadSigma = 0.0

        # Write detector debug images to local storage.  Very slow.
        # config.debug = True

        at_detector.setConfig(config)
        qtp = at_detector.QuadThresholdParameters()
        # The apriltag default is 5.  WPI overrides this
        # with 300, which prevents far-away detections.
        # So set it back to 5.
        # it seems not to make much difference in FPS
        # qtp.minClusterPixels = 300
        qtp.minClusterPixels = 5
        at_detector.setQuadThresholdParameters(qtp)
        at_detector.addFamily("tag36h11")
        return at_detector

    @staticmethod
    def estimator(tag_size: float, mtx: NDArray[np.float32]) -> AprilTagPoseEstimator:
        return AprilTagPoseEstimator(
            AprilTagPoseEstimator.Config(
                tag_size, mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2]
            )
        )
