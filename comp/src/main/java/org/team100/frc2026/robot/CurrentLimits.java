package org.team100.frc2026.robot;

public class CurrentLimits {
    public static final double INTAKE_STATOR = 50;
    public static final double INTAKE_SUPPLY = 30;

    public static final double INTAKE_EXTEND_STATOR = 50;
    public static final double INTAKE_EXTEND_SUPPLY = 30;

    public static final double CONVEYOR_STATOR = 50;
    public static final double CONVEYOR_SUPPLY = 30;

    // tuned 16/03/2026
    public static final double FEEDER_STATOR = 50;
    public static final double FEEDER_SUPPLY = 30;

    public static final double SHOOTER_STATOR = 60;
    public static final double SHOOTER_SUPPLY = 40;

    public static final double SHOOTER_HOOD_STATOR = 50;
    public static final double SHOOTER_HOOD_SUPPLY = 30;

    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    // https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L195
    // 2/26/25: Joel updated the supply limit to 90A, see 1678 code above. This is
    // essentially unlimited, so you'll need to run some other kind of limiter (e.g.
    // acceleration) to keep from browning out.
    // 3/14/26 lowered from 90 to 80
    public static final double DRIVE_STATOR = 90;
    public static final double DRIVE_SUPPLY = 60;
}
