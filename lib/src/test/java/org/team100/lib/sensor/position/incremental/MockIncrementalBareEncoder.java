package org.team100.lib.sensor.position.incremental;

/** Contains no logic. */
public class MockIncrementalBareEncoder implements IncrementalBareEncoder {
    public double position = 0;
    public double velocity = 0;

    @Override
    public double getVelocityRad_S() {
        return velocity;
    }

    @Override
    public double getUnwrappedPositionRad() {
        return position;
    }

    // @Override
    // public void reset() {
    //     //
    // }

    @Override
    public void close() {
        //
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double motorPositionRad) {
        position = motorPositionRad;
    }

    @Override
    public void periodic() {
        //
    }

}
