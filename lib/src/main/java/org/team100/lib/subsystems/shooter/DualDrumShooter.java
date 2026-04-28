package org.team100.lib.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DualDrumShooter extends Subsystem {
    /** Runs forever */
    Command spin();

    /** Runs forever. */
    Command stop();
}
