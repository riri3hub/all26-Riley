package org.team100.lib.targeting;

import edu.wpi.first.math.geometry.Rotation2d;

/** Firing solution for fixed muzzle velocity */
public record Solution(Rotation2d azimuth, double azimuthVelocity, Rotation2d elevation) {
}