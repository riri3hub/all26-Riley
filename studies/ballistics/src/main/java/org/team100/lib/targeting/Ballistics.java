package org.team100.lib.targeting;

/**
 * Given the shooter muzzle speed and elevation, predict landing range and time
 * of flight.
 */
public class Ballistics {
    /** gravity, m/s/s */
    private static final double G = 9.81;

    /**
     * The parabolic path is certainly wrong; this is only for testing.
     * 
     * @param speed     muzzle speed, meters/sec
     * @param elevation shooter elevation above horizontal, radians
     * @return landing solution
     */
    public static Interception parabolic(double speed, double elevation) {
        // https://en.wikipedia.org/wiki/Projectile_motion#Time_of_flight_or_total_time_of_the_whole_journey
        double vy = speed * Math.sin(elevation);
        double tof = 2 * vy / G;
        double vx = speed * Math.cos(elevation);
        double range = tof * vx;
        // In this case the target elevation is equal to the initial elevation
        return new Interception(range, tof, elevation);
    }

    /**
     * Computes the elevation required for the given range. This is useful to find
     * the starting point for optimization.
     * 
     * @param speed muzzle speed, meters/sec
     * @param range distance to target, meters
     * @return elevation required, radians
     */
    public static double parabolicElevation(double speed, double range) {
        return 0;
    }

    /**
     * Path with Newton drag (proportional to the square of velocity).
     * 
     * https://en.wikipedia.org/wiki/Projectile_motion#Trajectory_of_a_projectile_with_Newton_drag
     * 
     * @param speed     muzzle speed, meters/sec
     * @param elevation shooter elevation above horizontal, radians
     * @return landing solution
     */
    public static Interception newton(double speed, double elevation) {
        // this should use a precomputed lookup table.
        return new Interception(0, 0, 0);
    }

}
