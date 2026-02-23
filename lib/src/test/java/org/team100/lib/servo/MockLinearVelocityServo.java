package org.team100.lib.servo;

public class MockLinearVelocityServo implements LinearVelocityServo {
    double m_setpoint;

    @Override
    public void reset() {
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
    }

    @Override
    public void setVelocity(double setpoint) {
        m_setpoint = setpoint;
    }

    @Override
    public void setVelocity(double setpoint, double setpoint_2) {
        m_setpoint = setpoint;
    }

    @Override
    public double getVelocity() {
        return m_setpoint;
    }

    @Override
    public boolean atGoal() {
        return true;
    }

    @Override
    public void stop() {
    }

    @Override
    public double getDistance() {
        throw new UnsupportedOperationException("Unimplemented method 'getDistance'");
    }

    @Override
    public void periodic() {
    }

    @Override
    public void play(double freq) {
    }
}
