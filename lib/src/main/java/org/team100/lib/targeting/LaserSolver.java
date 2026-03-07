package org.team100.lib.targeting;

import java.util.Optional;

import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Solution for infinite muzzle velocity, for testing.
 * 
 * Azimuth is always the bearing.
 * Azimuth velocity is the apparent target motion.
 * Elevation is always zero.
 */
public class LaserSolver implements Solver {

    @Override
    public Optional<Solution> solve(
            ModelSE2 state,
            Translation2d targetPosition,
            GlobalVelocityR2 targetVelocity) {
        // Does not include target velocity in solution velocity.
        Rotation2d absoluteBearing = TargetUtil.absoluteBearing(
                state.pose().getTranslation(), targetPosition);
        double azimuthVelocity = TargetUtil.targetMotion(
                state, targetPosition);
        return Optional.of(new Solution(
                absoluteBearing,
                azimuthVelocity,
                Rotation2d.kZero));
    }

}
