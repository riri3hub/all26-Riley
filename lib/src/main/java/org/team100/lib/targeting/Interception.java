package org.team100.lib.targeting;

/**
 * When and where the ball intercepts the target.
 * 
 * @param range           in meters
 * @param tof             time of flight in seconds
 * @param targetElevation path angle at the target
 */
public record Interception(
        double range, double tof, double targetElevation) {

    @Override
    public String toString() {
        return String.format("range %6.3f, tof %6.3f, target elevation %6.3f",
                range, tof, targetElevation);
    }
}