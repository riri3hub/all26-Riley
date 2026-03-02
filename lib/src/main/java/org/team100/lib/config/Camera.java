package org.team100.lib.config;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents all the cameras. Some may be mounted on one robot, some on
 * another. Keep this up to date when you move cameras around.
 * Counter-clockwise is positive
 * x is forward, y is left, and z is up
 */
public enum Camera {
    //
    // Camera-bot cameras 2026
    //

    /**
     * Camera bot rear facing, 10.1.0.24
     */
    CAMERA_BACK("d44649628c20d4d4",
            fromCalibration(
                    new Transform3d(0.96, 0.031, 0.57, new Rotation3d(0, 0, Math.PI)),
                    new Transform3d(0.926, -0.293, -0.165, new Rotation3d(1.599, 0.03, 0.09)))),

    /**
     * Camera bot front facing
     */
    CAMERA_FRONT("8ddb2ed6c49a9bce",
            fromCalibration(
                    new Transform3d(0.96, 0.031, 0.57, new Rotation3d(0, 0, 0)),
                    new Transform3d(1.001, 0.267, -0.044, new Rotation3d(-1.57, -0.02, -0.05)))),

    //
    // TODO: cleanup the entries below
    //

    C("10000000a7c673d9",
            new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0, -Math.toRadians(10), 0))),
    /**
     * Delta shooter
     */
    SHOOTER("10000000a7a892c0",
            new Transform3d(
                    new Translation3d(-0.1265, 0.0682, 0.612),
                    new Rotation3d(0, Math.toRadians(-25), Math.toRadians(-2)))),

    /**
     * Delta amp-placer
     */
    RIGHTAMP("10000000caeaae82",
            new Transform3d(
                    new Translation3d(-0.1265, -0.1063625, 0.61),
                    new Rotation3d(0, Math.toRadians(-26), Math.toRadians(-63)))),

    /**
     * Delta amp-placer
     */
    LEFTAMP("100000004e0a1fb9",
            new Transform3d(
                    new Translation3d(-0.1265, 0.1532, 0.61),
                    new Rotation3d(0, Math.toRadians(-22), Math.toRadians(59)))),
    /**
     * Delta intake
     */
    GAME_PIECE("1000000013c9c96c",
            new Transform3d(
                    new Translation3d(-0.1265, 0.03, 0.61),
                    new Rotation3d(0, Math.toRadians(31.5), Math.PI))),

    /**
     * Right swerve
     */
    SWERVE_RIGHT("47403d5eafe002a9",
            new Transform3d(
                    new Translation3d(-0.261, -0.317, 0.217),
                    new Rotation3d(-0.146, 0.195, -0.508).unaryMinus().plus(new Rotation3d(0, 0, -Math.PI / 2)))),

    /**
     * Left swerve
     */
    SWERVE_LEFT("8132c256f63bbb4e",
            new Transform3d(
                    new Translation3d(-0.241, 0.297, 0.207),
                    new Rotation3d(0.07, 0.147, 0.52).unaryMinus().plus(new Rotation3d(0, 0, Math.PI / 2)))),

    /**
     * Funnel
     */
    FUNNEL("1e5acbaa5a7f9d10",
            new Transform3d(
                    new Translation3d(-0.034, -0.213, 0.902),
                    new Rotation3d(0.07, 0.48, 0.20).unaryMinus().plus(new Rotation3d(0, 0, Math.PI)))),

    /**
     * Coral reef right
     */
    CORAL_RIGHT("82c4c3fe4f941e96",
            new Transform3d(
                    new Translation3d(-0.29, -0.22, 0.89),
                    new Rotation3d(-0.14, -0.68, 0.23).unaryMinus().plus(new Rotation3d(0, 0, Math.PI)))),

    /**
     * For prototyping
     */
    DEV("364f07fb090a3bf7",
            new Transform3d(
                    new Translation3d(0.155, 0.295, 0.372),
                    new Rotation3d(0.05, -0.14, -.33).unaryMinus())),

    //
    // For unit tests.
    //
    ORIGIN("origin", new Transform3d()),
    TEST4("test4",
            new Transform3d(
                    new Translation3d(0, 0, 1),
                    new Rotation3d(0, 0, 0))),
    TEST5("test5",
            new Transform3d(
                    new Translation3d(0, 0.1, 1),
                    new Rotation3d(0, 0, 0))),
    TEST6("test6",
            new Transform3d(
                    new Translation3d(0.198, 0.284, 0.811),
                    new Rotation3d(-0.043, -0.705, 0.254).unaryMinus())),
    TEST7("test7",
            new Transform3d(
                    new Translation3d(1, 0, 1.368),
                    new Rotation3d(0, -0.523, 0))),
    TEST7A("test7a",
            new Transform3d(
                    new Translation3d(0, 0, 1.368),
                    new Rotation3d(0, -0.523, 0))),
    TEST8("test8",
            new Transform3d(
                    new Translation3d(),
                    new Rotation3d(0, Math.PI / 4, 0))),
    TEST9("test9",
            new Transform3d(
                    new Translation3d(),
                    new Rotation3d(0, Math.PI / 6, 0))),

    //
    // Four directions for simulation, a bit off the floor.
    //
    SIM0("sim0",
            new Transform3d(
                    new Translation3d(0, 0, 0.75),
                    new Rotation3d(0, 0, 0))),
    SIM1("sim1",
            new Transform3d(
                    new Translation3d(0, 0, 0.75),
                    new Rotation3d(0, 0, Math.PI / 2))),
    SIM2("sim2",
            new Transform3d(
                    new Translation3d(0, 0, 0.75),
                    new Rotation3d(0, 0, Math.PI))),
    SIM3("sim3",
            new Transform3d(
                    new Translation3d(0, 0, 0.75),
                    new Rotation3d(0, 0, -Math.PI / 2))),

    //
    // This should never be used
    //
    UNKNOWN(null, new Transform3d());

    private static Map<String, Camera> cameras = new HashMap<>();
    static {
        for (Camera i : Camera.values()) {
            cameras.put(i.m_serialNumber, i);
        }
    }
    private String m_serialNumber;
    private Transform3d m_Offset;

    private Camera(String serialNumber, Transform3d offset) {
        m_serialNumber = serialNumber;
        m_Offset = offset;
    }

    public static Camera get(String serialNumber) {
        if (cameras.containsKey(serialNumber))
            return cameras.get(serialNumber);
        // throw new IllegalArgumentException(
        // String.format("unknown camera serial number: \s", serialNumber));
        System.out.println("#############################################");
        System.out.println("###");
        System.out.printf("### Unknown camera serial number: %s\n", serialNumber);
        System.out.println("###");
        System.out.println("#############################################");
        return UNKNOWN;
    }

    public Transform3d getOffset() {
        return m_Offset;
    }

    public String getSerial() {
        return m_serialNumber;
    }

    /**
     * Use this to calibrate the cameras. Set the transform to identity, set a tag
     * in a known location ("robot to tag"), and enter what the camera thinks the
     * tag pose is -- this appears on glass as "camera to tag".
     */
    public static Transform3d fromCalibration(Transform3d robotToTag, Transform3d cameraToTag) {
        return robotToTag.plus(cameraToTag.inverse());
    }
}
