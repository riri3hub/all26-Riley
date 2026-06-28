package frc.robot;

import java.io.IOException;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Use the apriltag corners to derive a pose estimate.
 * 
 * This is a half-step to GTSAM; it's not better than
 * doing the pose estimation on the Raspberry Pi.
 */
public class PoseFromCorners {
    static {
        try {
            OpenCvLoader.forceLoad();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    private static final MatOfPoint2f CORNERS_FOR_HOMOGRAPHY = new MatOfPoint2f(
            new Point(-1, 1),
            new Point(1, 1),
            new Point(1, -1),
            new Point(-1, -1));

    AprilTagPoseEstimator estimator;

    public PoseFromCorners() {
        // TODO: make a lookup table for configs
        AprilTagPoseEstimator.Config conf = new AprilTagPoseEstimator.Config(
                0.1651, 935, 935, 550, 310);
        estimator = new AprilTagPoseEstimator(conf);
    }

    public Transform3d pose(double[] corners) {
        double[] homography = getOpenCvHomographyArray(corners);
        return estimator.estimate(homography, corners);
    }

    private double[] getOpenCvHomographyArray(double[] corners) {
        MatOfPoint2f dstPoints = new MatOfPoint2f(
                new Point(corners[0], corners[1]),
                new Point(corners[2], corners[3]),
                new Point(corners[4], corners[5]),
                new Point(corners[6], corners[7]));
        Mat openCvHomographyMat = Calib3d.findHomography(CORNERS_FOR_HOMOGRAPHY, dstPoints);
        double[] openCvHomographyArray = new double[9];
        openCvHomographyMat.get(0, 0, openCvHomographyArray);
        return openCvHomographyArray;
    }

}
