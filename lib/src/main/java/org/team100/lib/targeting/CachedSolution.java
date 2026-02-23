package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Using (moving) robot state and (fixed) target position, this maintains a
 * solution that may be used by multiple consumers.
 */
public class CachedSolution implements Supplier<Optional<Solution>> {
    private final Supplier<ModelSE2> m_state;
    private final Supplier<Optional<Translation2d>> m_target;
    private final Solver m_solver;
    private final ObjectCache<Optional<Solution>> m_cache;
    private final DoubleArrayLogger m_log_target;

    public CachedSolution(
            LoggerFactory fieldLogger,
            Supplier<ModelSE2> state,
            Supplier<Optional<Translation2d>> target,
            Solver solver) {
        m_state = state;
        m_target = target;
        m_solver = solver;
        m_cache = Cache.of(this::solve);
        m_log_target = fieldLogger.doubleArrayLogger(Level.TRACE, "target");
    }

    @Override
    public Optional<Solution> get() {
        return m_cache.get();
    }

    private Optional<Solution> solve() {
        // This is the *current* state
        ModelSE2 state = m_state.get();

        // We're aiming at the *next* timestep.
        state = state.evolve(TimedRobot100.LOOP_PERIOD_S);

        Optional<Translation2d> oTargetPosition = m_target.get();

        if (oTargetPosition.isEmpty())
            return Optional.empty();

        Translation2d targetPosition = oTargetPosition.get();
        
        m_log_target.log(() -> new double[] {
                targetPosition.getX(),
                targetPosition.getY(),
                0 });
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;

        return m_solver.solve(
                state, targetPosition, targetVelocity);
    }

}
