package org.team100.lib.visualization;

import java.util.function.Supplier;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.Drag;

import edu.wpi.first.math.geometry.Rotation2d;

public class BallFactory {

    public static Ball get2d(
            LoggerFactory field,
            Supplier<ModelSE2> robot,
            Supplier<Rotation2d> azimuth,
            double speed) {
        return new BallR2(field, robot, azimuth, speed);

    }

    public static Ball get3d(
            LoggerFactory field,
            Supplier<ModelSE2> robot,
            Supplier<Rotation2d> azimuth,
            Supplier<Rotation2d> elevation,
            double speed,
            double omega) {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        return new BallR3(field, d, robot, azimuth, elevation, speed, omega);
    }

}
