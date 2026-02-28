// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    private final Sync sync;

    public Robot() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer();
        sync = new Sync(inst);
    }

    @Override
    public void robotPeriodic() {
        sync.run();
    }
}
