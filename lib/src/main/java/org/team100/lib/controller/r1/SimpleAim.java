package org.team100.lib.controller.r1;

import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelR1Logger;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.TargetUtil;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Controls omega to always aim at a target.
 * 
 * Use this to override user or trajectory omega input.
 * 
 * Does not account for relative target motion.
 */
public class SimpleAim {
    private final DoubleSupplier m_maxOmega;
    private final FeedbackR1 m_thetaController;

    private final ModelR1Logger m_log_thetaGoal;
    private final DoubleLogger m_log_apparent_motion;
    private final DoubleArrayLogger m_log_target;
    private final ModelR1Logger m_log_goal;
    private final DoubleLogger m_log_thetaFB;
    private final DoubleLogger m_log_thetaFF;
    private final DoubleLogger m_log_omega;

    public SimpleAim(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            DoubleSupplier maxOmega,
            FeedbackR1 thetaController) {
        LoggerFactory log = parent.type(this);
        m_log_goal = log.ModelR1Logger(Level.TRACE, "goal");
        m_log_thetaFB = log.doubleLogger(Level.TRACE, "thetaFB");
        m_log_thetaFF = log.doubleLogger(Level.TRACE, "thetaFF");
        m_log_omega = log.doubleLogger(Level.TRACE, "omega");
        m_log_target = fieldLogger.doubleArrayLogger(Level.TRACE, "target");
        m_log_apparent_motion = log.doubleLogger(Level.TRACE, "apparent motion");
        m_log_thetaGoal = log.ModelR1Logger(Level.TRACE, "theta goal");
        m_maxOmega = maxOmega;
        m_thetaController = thetaController;
    }

    public void reset() {
        m_thetaController.reset();
    }

    public double getOmega(ModelSE2 state, Translation2d target) {
        // Show the target.
        m_log_target.log(() -> new double[] {
                target.getX(),
                target.getY(),
                0 });

        // Goal omega should match the target's apparent motion.
        double azimuthVelocity = TargetUtil.targetMotion(state, target);
        m_log_apparent_motion.log(() -> azimuthVelocity);

        double unwrappedBearing = TargetUtil.unwrappedAbsoluteBearing(state.pose(), target);

        ModelR1 thetaGoal = new ModelR1(unwrappedBearing, azimuthVelocity);

        return getOmega(state, thetaGoal);
    }

    public Double getOmega(ModelSE2 state, ModelR1 thetaGoal) {
        m_log_thetaGoal.log(() -> thetaGoal);
        ModelR1 theta = state.theta();
        double thetaFB = getThetaFB(theta, thetaGoal);
        double thetaFF = getThetaFF(thetaGoal);
        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_maxOmega.getAsDouble(),
                m_maxOmega.getAsDouble());
        m_log_omega.log(() -> omega);
        return omega;
    }

    private double getThetaFB(ModelR1 theta, ModelR1 thetaGoal) {
        // wrap correctly
        double robotYaw = theta.x();
        double goalYaw = Math100.getMinDistance(robotYaw, thetaGoal.x());
        ModelR1 goal = new ModelR1(goalYaw, thetaGoal.v());
        m_log_goal.log(() -> goal);
        // compute feedback
        double thetaFB = m_thetaController.calculate(theta, goal);
        m_log_thetaFB.log(() -> thetaFB);
        return thetaFB;
    }

    private double getThetaFF(ModelR1 thetaGoal) {
        double thetaFF = thetaGoal.v();
        m_log_thetaFF.log(() -> thetaFF);
        return thetaFF;
    }
}
