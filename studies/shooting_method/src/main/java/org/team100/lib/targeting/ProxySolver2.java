package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.frc2026.field.FieldConstants2026ForStudy;
import org.team100.lib.geometry.StateR2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Provides a solver based on the dashboard widget.
 */
public class ProxySolver2 implements Solver {
    private final SendableChooser<Solver> m_chooser;

    public ProxySolver2(DoubleFunction<Optional<FiringParameters>> rangeToParams) {
        m_chooser = new NamedChooser<>("Target Solver");
        addTOFRecursion(rangeToParams);
        addLaser(rangeToParams);
        SmartDashboard.putData(m_chooser);
    }

    /** Keeps tests from conflicting. */
    public void close() {
        m_chooser.close();
    }

    /**
     * Real 3d trajectory.
     * 
     * Iterates on predicted TOF using inverse map.
     */
    private void addTOFRecursion(DoubleFunction<Optional<FiringParameters>> rangeToParams) {
        m_chooser.addOption("TOFR", new TimeOfFlightRecursion(rangeToParams, 0.01));
    }

    /**
     * Two-dimensional solution.
     * 
     * For testing, using the flashlight.
     */
    private void addLaser(DoubleFunction<Optional<FiringParameters>> rangeToParams) {
        m_chooser.setDefaultOption("Laser", new LaserSolver(rangeToParams));
    }

    @Override
    public Optional<Solution> solve(ModelSE2 state, StateR2 target) {
        return m_chooser.getSelected().solve(state, target);
    }

    ////////////////////////////////////////////////
    ///
    /// PARKING LOT
    ///

    /**
     * Real 3d trajectory.
     * 
     * Iterates on azimuth and elevation using direct map.
     * 
     * TODO: Seems not to work. Make it work someday.
     */
    @SuppressWarnings("unused")
    private void addShootingMethod(
            double muzzleVelocityM_S,
            double omegaRad_S) {
        Drag dragModel = FieldConstants2026ForStudy.FUEL_DRAG;
        RangeSolver rangeSolver = new RangeSolver(
                dragModel,
                FieldConstants2026.HUB.getZ(),
                FieldConstants2026.HUB_ELEVATION,
                0.001);
        RangeCache rangeCache = new RangeCache(
                rangeSolver, muzzleVelocityM_S, omegaRad_S);
        double tolerance = 0.01;
        double initialElevation = 0.1;
        m_chooser.addOption("Shooting",
                new ShootingMethod(
                        rangeCache, 0.1, 1.4, tolerance, initialElevation));
    }
}
