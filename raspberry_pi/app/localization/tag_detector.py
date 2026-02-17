""" A wrapper for the AprilTag detector. """

# pylint: disable=E0611,R0902,R0903,R0913,R0914,W0212

import os
from typing import cast

import ntcore
import numpy as np
from cv2 import imwrite, undistort, undistortImagePoints
from numpy.typing import NDArray
from robotpy_apriltag import (AprilTagDetection, AprilTagDetector,
                              AprilTagPoseEstimator)
from typing_extensions import override

from app.camera.camera_protocol import Camera, Request, Size
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.display import Display
from app.network.network import Blip24, Network

Mat = NDArray[np.uint8]


class TagDetector(Interpreter):
    IMAGE_DIR = "images"

    def __init__(
        self,
        identity: Identity,
        cam: Camera,
        camera_num: int,
        display: Display,
        network: Network,
        debug: bool = False,
    ) -> None:
        """debug is very slow, writes apriltag detector debug images into the same filenames over and over, and also writes timestamped images for later analysis"""
        self.identity = identity
        self.cam = cam
        self.camera_num = camera_num
        self.display = display
        self.network = network
        self.debug = debug

        print("\n*** Interpreter: TagDetector")

        self.mtx: Mat = cam.get_intrinsic()
        self.dist: Mat = cam.get_dist()

        size: Size = cam.get_size()
        self.width: int = size.width
        self.height: int = size.height

        self.y_len = self.width * self.height

        self.at_detector = AprilTagDetector()
        config = self.at_detector.Config()
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
        config.quadDecimate = 4.0
        config.refineEdges = True

        # stddev of the blur kernel in pixels: seems to help with small tags
        config.quadSigma = 0.0
        if self.debug:
            config.debug = True
        self.at_detector.setConfig(config)
        qtp = self.at_detector.QuadThresholdParameters()
        # The apriltag default is 5.  WPI overrides this
        # with 300, which prevents far-away detections.
        # So set it back to 5.
        # it seems not to make much difference in FPS
        # qtp.minClusterPixels = 300
        qtp.minClusterPixels = 5
        self.at_detector.setQuadThresholdParameters(qtp)
        self.at_detector.addFamily("tag36h11")

        if identity == Identity.DIST_TEST:
            # the distortion rig uses a 33 mm, 20% scale, tag.
            tag_size = 0.033
        elif identity == Identity.FUNNEL:
            # for rotation testing rig
            tag_size = 0.059
        else:
            # normal tag size is 6.5 inches
            tag_size = 0.1651
        self.estimator = AprilTagPoseEstimator(
            AprilTagPoseEstimator.Config(
                tag_size,
                self.mtx[0, 0],
                self.mtx[1, 1],
                self.mtx[0, 2],
                self.mtx[1, 2],
            )
        )

        # TODO: move the identity part of this path to the Network object
        path = "vision/" + identity.value + "/" + str(camera_num)
        # network output for the tag sightings
        self._blips = network.get_blip_sender(path + "/blips")
        # network output for camera FPS
        self._fps = network.get_double_sender(path + "/fps")
        self._temp = network.get_double_sender(path + "/temp")
        self._offset = network.get_double_sender(path + "/offset")
        self._now = network.get_double_sender(path + "/now")
        self._servernow = ntcore.NetworkTableInstance.getDefault().getDoubleTopic("servernow").subscribe(0)
        self._nowdiff = network.get_double_sender(path + "/nowdiff")
        # to keep track of images to write
        self.img_ts_sec = 0
        if self.debug:
            # make a place to put example images
            if not os.path.exists(TagDetector.IMAGE_DIR):
                os.mkdir(TagDetector.IMAGE_DIR)

    @override
    def analyze(self, req: Request) -> None:
        with req.yuv() as buffer:
            # truncate, ignore chrominance. this makes a view, very fast (300 ns)
            img = cast(
                Mat,
                np.frombuffer(buffer, dtype=np.uint8, count=self.y_len),  # type:ignore
            )

            # this  makes a view, very fast (150 ns)
            img: Mat = img.reshape((self.height, self.width))  # type:ignore

            if self.debug:
                # Write some of the files for later analysis (e.g. calibration)
                # To retrieve these files, use, e.g.:
                # scp pi@10.1.0.11:images/* .
                # These will accumulate forever so remember to clean it out:
                # ssh pi@10.1.0.11 "rm images/img*"
                now_us = ntcore._now()  # pylint:disable=W0212
                now_s = now_us // 1000000  # once per second
                if now_s > self.img_ts_sec:
                    self.img_ts_sec = now_s
                    filename = TagDetector.IMAGE_DIR + "/img" + str(now_s) + ".png"
                    imwrite(filename, img)

            #######
            #
            # uncomment this line to undistort the whole image, for debugging
            # img = undistort(img, self.mtx, self.dist)
            #
            #######

            # is detection the slow part?
            # result: list[AprilTagDetection] = []

            result: list[AprilTagDetection] = self.at_detector.detect(img.data)

            blips: list[Blip24] = []
            result_item: AprilTagDetection
            for result_item in result:
                if result_item.getHamming() > 0:
                    continue

                # UNDISTORT EACH ITEM
                # undistortPoints is at least 10X faster than undistort on the whole image.
                # the order here is:
                # lower left
                # lower right
                # upper right
                # upper left
                corners: tuple[
                    float, float, float, float, float, float, float, float
                ] = result_item.getCorners((0, 0, 0, 0, 0, 0, 0, 0))

                # undistortPoints wants [[x0,y0],[x1,y1],...]
                pairs = np.reshape(corners, [4, 2])
                pairs = undistortImagePoints(pairs, self.mtx, self.dist)

                # the estimator wants [x0, y0, x1, y1, ...]
                # pairs has an extra dimension, so redo it:
                corners = (
                    pairs[0][0][0],
                    pairs[0][0][1],
                    pairs[1][0][0],
                    pairs[1][0][1],
                    pairs[2][0][0],
                    pairs[2][0][1],
                    pairs[3][0][0],
                    pairs[3][0][1],
                )

                homography = result_item.getHomography()
                pose = self.estimator.estimate(homography, corners)

                blips.append(Blip24(result_item.getId(), pose))
                self.display.tag(img, result_item, pose)

            delay_us = req.delay_us()

            # send sightings to network
            self._blips.send(blips, delay_us)
           
            # send camera FPS to network
            fps = req.fps()
            self._fps.send(fps, delay_us)
            

            # must flush!  otherwise 100ms update rate.
            self.network.flush()

            # log the CPU temperature in C
            # raspberry pi throttles at 80
            with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="ascii") as f:
                raw_temp = int(f.read().strip())
                temp_c = raw_temp / 1000
                self._temp.send(temp_c, 0)

            offset = ntcore.NetworkTableInstance.getDefault().getServerTimeOffset()
            if offset is not None:
                self._offset.send((float)(offset/1000000), 0)

            now = ntcore._now()
            self._now.send((float)(now), 0)
            servernow = self._servernow.get()
            self._nowdiff.send(servernow - now, 0)

            # do the drawing (after the NT payload is written)
            # to minimize latency
            # none of this is particularly fast or important for prod.

            self.display.text(img, f"FPS {fps:2.0f}", (10, 80))
            self.display.text(img, f"DELAY (ms) {delay_us/1000:2.0f}", (10, 160))
            self.display.put(img)
