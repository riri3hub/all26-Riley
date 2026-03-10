package org.team100.lib.targeting;

import java.util.Optional;

import org.team100.lib.geometry.StateR2;
import org.team100.lib.state.ModelSE2;

/** Interface for shooting solvers, for moving robot and/or target */
public interface Solver {
    /**
     * Does not necessarily return the "short way around". Consumers should do their
     * own post-process to find an Euler angle that suits their state.
     */
    Optional<Solution> solve(ModelSE2 robot, StateR2 target);
}