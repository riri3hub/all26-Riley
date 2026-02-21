package org.team100.lib.servo;

/**
 * This is just for tests and illustration.
 */
public class MockAngularPositionServo implements AngularPositionServo {

    @Override
    public void reset() {
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
    }

    @Override
    public void setPositionProfiled(double goalRad, double torqueNm) {
    }

    @Override
    public void setPositionDirect(double goalRad, double velocityRad_S, double torqueNm) {
    }

    @Override
    public double getWrappedPositionRad() {
        return 0;
    }

    @Override
    public double getUnwrappedPositionRad() {
        return 0;
    }

    @Override
    public boolean atSetpoint() {
        return false;
    }

    @Override
    public boolean profileDone() {
        return false;
    }

    @Override
    public boolean atGoal() {
        return false;
    }

    @Override
    public void stop() {
    }

    @Override
    public void close() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void play(double freq) {
    }

    @Override
    public void actuateWithProfile(double unwrappedGoalX, double torqueNm) {
    }

}
