package org.team100.lib.targeting;

/**
 * Shooting solution including time of flight.
 * 
 * @param range     (m) to target center, in 2d, measured on the floor
 * @param speed     (m/s) shooter drum speed
 * @param elevation (rad) of the hood, not the ball path
 * @param tof       (sec) measured with a stopwatch
 */
public record FiringParameters(
        double range, double speed, double elevation, double tof) {

}
