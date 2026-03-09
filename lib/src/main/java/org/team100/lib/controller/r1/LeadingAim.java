package org.team100.lib.controller.r1;

import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelR1Logger;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.Solution;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;

/**
 * Controls omega to hit a moving target from a moving platform.
 * 
 * Use this to override user or trajectory omega input.
 * 
 * Leads relative target motion.
 */
public class LeadingAim {
    private final DoubleSupplier m_maxOmega;
    private final FeedbackR1 m_thetaController;

    private final DoubleLogger m_log_apparent_motion;
    private final ModelR1Logger m_log_goal;
    private final DoubleLogger m_log_thetaFB;
    private final DoubleLogger m_log_thetaFF;
    private final DoubleLogger m_log_omega;

    // feedback operates on the previous goal;
    // feedforward should suffice for the next goal.
    private ModelR1 m_goal;

    public LeadingAim(
            LoggerFactory parent,
            DoubleSupplier maxOmega,
            FeedbackR1 thetaController) {
        LoggerFactory log = parent.type(this);
        m_log_goal = log.ModelR1Logger(Level.TRACE, "goal");
        m_log_thetaFB = log.doubleLogger(Level.TRACE, "thetaFB");
        m_log_thetaFF = log.doubleLogger(Level.TRACE, "thetaFF");
        m_log_omega = log.doubleLogger(Level.TRACE, "omega");
        m_log_apparent_motion = log.doubleLogger(Level.TRACE, "apparent motion");
        m_maxOmega = maxOmega;
        m_thetaController = thetaController;
    }

    /** Sets goal to measurement, resets controller. */
    public void reset(ModelSE2 state) {
        m_goal = state.theta();
        m_thetaController.reset();
    }

    public Double getOmega(ModelSE2 state, Solution solution) {

        double thetaFB = getThetaFB(state);
        double thetaFF = getThetaFF(solution, state);

        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_maxOmega.getAsDouble(),
                m_maxOmega.getAsDouble());
        m_log_omega.log(() -> omega);

        return omega;

    }

    /** Feedback uses the previous goal. */
    private double getThetaFB(ModelSE2 state) {
        double thetaFB = m_thetaController.calculate(state.theta(), m_goal);
        m_log_thetaFB.log(() -> thetaFB);
        return thetaFB;
    }

    private double getThetaFF(Solution solution, ModelSE2 state) {

        double robotYaw = state.pose().getRotation().getRadians();
        double goalYaw = Math100.getMinDistance(robotYaw, solution.azimuth().getRadians());
        // m_goal = new ModelR1(goalYaw, 0);
        double targetMotion = solution.azimuthVelocity();
        m_log_apparent_motion.log(() -> targetMotion);

        // Assign the new goal
        m_goal = new ModelR1(goalYaw, targetMotion);
        m_log_goal.log(() -> m_goal);

        double thetaFF = m_goal.v();
        m_log_thetaFF.log(() -> thetaFF);

        return thetaFF;
    }

}
