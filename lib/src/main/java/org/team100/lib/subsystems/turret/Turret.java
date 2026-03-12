package org.team100.lib.subsystems.turret;

import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;

import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.geometry.StateR2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.FiringParameters;
import org.team100.lib.targeting.LaserSolver;
import org.team100.lib.targeting.Solution;
import org.team100.lib.targeting.Solver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Attempts to maintain aim. Demonstrates "spotting".
 * 
 * It provides Field2d visualization of the turret using the name "turret".
 * 
 * There's a version of this in the shooting_method study that has a few more methods.
 */
public class Turret extends SubsystemBase {
    private static final double GEAR_RATIO = 100;
    private static final double MIN_POSITION = -3;
    private static final double MAX_POSITION = 3;

    private final DoubleArrayLogger m_log_field_turret;
    private final Supplier<ModelSE2> m_state;
    private final Supplier<Translation2d> m_target;
    private final AngularPositionServo m_pivot;
    private final AngularPositionServo m_elevation;
    private final Solver m_laser;
    private boolean m_aiming;

    /**
     * @param parent Log
     * @param field  Field2d log
     * @param state  Current robot pose, from the pose estimator.
     * @param target From the target designator; can change at any time.
     */
    public Turret(
            LoggerFactory parent,
            LoggerFactory field,
            DoubleFunction<Optional<FiringParameters>> rangeToParams,
            Supplier<ModelSE2> state,
            Supplier<Translation2d> target,
            double speed) {
        LoggerFactory log = parent.type(this);
        m_log_field_turret = field.doubleArrayLogger(Level.COMP, "turret");
        m_state = state;
        m_target = target;
        m_pivot = pivot(log);
        m_elevation = elevation(log);
        m_laser = new LaserSolver(rangeToParams);
        m_aiming = false;
    }

    private static AngularPositionServo pivot(LoggerFactory log) {
        ProfileR1 profile = new TrapezoidProfileR1(log, 5, 10, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
        IncrementalBareEncoder encoder = motor.encoder();
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                log, encoder, GEAR_RATIO);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, GEAR_RATIO, MIN_POSITION, MAX_POSITION);
        AngularPositionServo pivot = new OnboardAngularPositionServo(
                log, mech, ref, feedback);
        pivot.reset();
        return pivot;
    }

    private static AngularPositionServo elevation(LoggerFactory log) {
        ProfileR1 profile = new TrapezoidProfileR1(log, 5, 10, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
        IncrementalBareEncoder encoder = motor.encoder();
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                log, encoder, GEAR_RATIO);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, GEAR_RATIO, 0, Math.PI / 2);
        AngularPositionServo pivot = new OnboardAngularPositionServo(
                log, mech, ref, feedback);
        pivot.reset();
        return pivot;
    }

    public boolean onTarget() {
        return m_aiming && m_pivot.atGoal();
    }

    /** Absolute turret rotation */
    public Rotation2d getAzimuth() {
        Rotation2d relative = new Rotation2d(m_pivot.getWrappedPositionRad());
        return m_state.get().rotation().plus(relative);
    }

    public Rotation2d getElevation() {
        return new Rotation2d(m_elevation.getWrappedPositionRad());
    }

    private void moveToAim() {
        m_aiming = true;
        Optional<Solution> soln = getSolution();
        if (soln.isEmpty()) {
            // no solution is possible, don't do anything
            m_pivot.stop();
            m_elevation.stop();
            return;
        }
        Rotation2d absoluteBearing = soln.get().azimuth();
        Rotation2d relativeBearing = absoluteBearing.minus(m_state.get().rotation());
        m_pivot.setPositionProfiled(relativeBearing.getRadians(), 0);
        m_elevation.setPositionProfiled(soln.get().parameters().elevation(), 0);
    }

    private Optional<Solution> getSolution() {
        // For shining a flashlight at the target.
        return getAbsoluteBearingInstantaneous();
    }

    /**
     * Compute absolute bearing from robot to target without compensating for the
     * motion of either one.
     */
    private Optional<Solution> getAbsoluteBearingInstantaneous() {
        StateR2 target = new StateR2(m_target.get(), GlobalVelocityR2.ZERO);
        return m_laser.solve(m_state.get(), target);
    }

    private void stopAiming() {
        m_aiming = false;
        m_pivot.stop();
    }

    ////////////////////////////////////////////////////////
    ///
    /// COMMANDS

    public Command aim() {
        return run(this::moveToAim);
    }

    public Command stop() {
        return run(this::stopAiming);
    }

    @Override
    public void periodic() {
        m_pivot.periodic();
        m_log_field_turret.log(this::poseArray);
    }

    private double[] poseArray() {
        Pose2d pose = m_state.get().pose();
        return new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().plus(
                        new Rotation2d(m_pivot.getWrappedPositionRad()))
                        .getDegrees() };
    }

}
