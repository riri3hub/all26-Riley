package org.team100.lib.dynamics.se2;

/**
 * @param fx force in x, N
 * @param fy force in y, N
 * @param t  torque, Nm
 */
public record SE2Torque(double fx, double fy, double t) {

}
