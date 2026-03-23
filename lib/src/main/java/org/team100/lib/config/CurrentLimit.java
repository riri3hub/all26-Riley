package org.team100.lib.config;

/**
 * The stator limit is proportional to the applied torque. Use this to regulate
 * the force produced by the motor. This is usually only useful when the motor
 * is stalled, e.g. the drive wheels in a pushing match, or a manipulator
 * holding on to something.
 * 
 * The supply limit is essentially a power limit, since the supply voltage is
 * (very roughly) constant. Use this to moderate the destruction of the robot
 * battery.
 * 
 * @param stator limit in amps
 * @param supply limit in amps
 */
public record CurrentLimit(double stator, double supply) {
}
