package org.team100.lib.targeting;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Firing solution for fixed muzzle velocity
 * 
 * @param azimuth         field-relative
 * @param azimuthVelocity for feed-forward control for moving shooter or target
 * @param elevation       above the horizontal
 */
public record Solution(
        Rotation2d azimuth,
        double azimuthVelocity,
        Rotation2d elevation) {
}
