package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.trajectory.constraint.TimingConstraint;
import org.team100.lib.trajectory.path.PathSE2;
import org.team100.lib.trajectory.path.PathSE2Entry;
import org.team100.lib.trajectory.path.PathSE2Point;
import org.team100.lib.util.Math100;

/**
 * Given a path, produces a trajectory, which includes the path and adds a
 * schedule.
 */
public class TrajectorySE2Factory {
    public static final boolean DEBUG = false;
    private static final double EPSILON = 1e-6;

    /** Defaults to make the constraints set the actual. */
    private static final double HIGH_V = 100;
    private static final double HIGH_ACCEL = 1000;

    private final List<TimingConstraint> m_constraints;

    public TrajectorySE2Factory(List<TimingConstraint> constraints) {
        m_constraints = constraints;
    }

    /**
     * Assigns a time to each point in the path.
     * 
     * Output is these same points with time.
     */
    public TrajectorySE2 fromPath(PathSE2 path, double start_vel, double end_vel) {
        double[] distances = distances(path);
        double[] velocities = velocities(path, start_vel, end_vel, distances);
        double[] accels = accels(distances, velocities);
        double[] runningTime = runningTime(distances, velocities, accels);
        List<TrajectorySE2Entry> timedStates = timedStates(path, velocities, accels, runningTime);
        return new TrajectorySE2(timedStates, m_constraints);
    }

    /////////////////////////////////////////////////////////////////////////////////////

    /**
     * Computes the length of each segment, as if it were a straight line, and
     * accumulates.
     */
    private double[] distances(PathSE2 path) {
        int n = path.length();
        double distances[] = new double[n];
        for (int i0 = 0; i0 < n - 1; ++i0) {
            int i1 = i0 + 1;
            double segmentLength = path.getEntry(i1).point().distanceCartesian(path.getEntry(i0).point());
            distances[i1] = segmentLength + distances[i0];
        }
        return distances;
    }

    /**
     * Assigns a velocity to each sample, using velocity, accel, and decel
     * constraints.
     */
    private double[] velocities(
            PathSE2 path, double start_vel, double end_vel, double[] distances) {
        double velocities[] = new double[path.length()];
        forward(path, start_vel, distances, velocities);
        backward(path, end_vel, distances, velocities);
        if (start_vel > velocities[0]) {
            System.out.printf("WARNING: start velocity %f is higher than constrained velocity %f\n",
                    start_vel, velocities[0]);
        }
        return velocities;
    }

    /**
     * Computes average accel based on distance of each arc and velocity at each
     * point.
     * 
     * Accel is attached to the *start* of each arc ([i] not [i+1])
     * 
     * The very last accel is always zero, but it's never used since it describes
     * samples off the end of the trajectory.
     */
    private double[] accels(double[] distances, double[] velocities) {
        int n = distances.length;
        double[] accels = new double[n];
        for (int i0 = 0; i0 < n - 1; ++i0) {
            int i1 = i0 + 1;
            double arcLength = distances[i1] - distances[i0];
            accels[i0] = Math100.accel(velocities[i0], velocities[i1], arcLength);
        }
        return accels;
    }

    /**
     * Computes duration of each arc and accumulate. Assigns a time to each point.
     */
    private double[] runningTime(double[] distances, double[] velocities, double[] accels) {
        int n = distances.length;
        double[] runningTime = new double[n];
        for (int i0 = 0; i0 < n - 1; ++i0) {
            int i1 = i0 + 1;
            double arcLength = distances[i1] - distances[i0];
            double dt = dt(velocities[i0], velocities[i1], arcLength, accels[i0]);
            runningTime[i1] = runningTime[i0] + dt;
        }
        return runningTime;
    }

    /**
     * Creates a list of timed states.
     */
    private List<TrajectorySE2Entry> timedStates(
            PathSE2 path, double[] velocities, double[] accels, double[] runningTime) {
        int n = path.length();
        List<TrajectorySE2Entry> timedStates = new ArrayList<>(n);
        for (int i = 0; i < n; ++i) {
            PathSE2Entry pe = path.getEntry(i);
            TrajectorySE2Entry te = new TrajectorySE2Entry(
                    pe.parameter(),
                    new TrajectorySE2Point(pe.point(), runningTime[i], velocities[i], accels[i]));
            timedStates.add(te);
        }
        return timedStates;
    }

    /**
     * Computes velocities[i+1] using velocity and acceleration constraints
     * referencing the state at i.
     */
    private void forward(
            PathSE2 path, double start_vel, double[] distances, double[] velocities) {
        int n = path.length();
        velocities[0] = start_vel;
        for (int i0 = 0; i0 < n - 1; ++i0) {
            int i1 = i0 + 1;
            if (DEBUG)
                System.out.printf("FWD i %d\n", i0);
            double arclength = distances[i1] - distances[i0];
            if (Math.abs(arclength) < EPSILON) {
                if (DEBUG)
                    System.out.printf("i %d zero arc\n", i0);
                // zero-length arcs have the same state at both ends
                velocities[i1] = velocities[i0];
                break;
            }
            // velocity constraint depends only on state
            double maxVelocity = maxVelocity(path.getEntry(i1).point());
            if (DEBUG)
                System.out.printf("maxV i %d maxV %f\n", i1, maxVelocity);
            // start with the maximum velocity
            velocities[i1] = maxVelocity;
            // reduce velocity to fit under the acceleration constraint
            double impliedAccel = Math100.accel(velocities[i0], velocities[i1], arclength);
            double maxAccel = maxAccel(path.getEntry(i0).point(), velocities[i0]);
            if (impliedAccel > maxAccel) {
                velocities[i1] = Math100.v1(velocities[i0], maxAccel, arclength);
                if (DEBUG) {
                    System.out.printf("adjust vi+1 %f\n", velocities[i1]);
                }
            }
            if (DEBUG) {
                System.out.printf("FWD i0 %d vi0 %f vi1 %f maxA %f impliedA %f\n",
                        i0, velocities[i0], velocities[i1], maxAccel, impliedAccel);
            }
        }
    }

    /**
     * Adjusts velocities[i] for decel constraint referencing the state at i+1, and
     * then again for i.
     */
    private void backward(
            PathSE2 path, double end_vel, double[] distances, double[] velocities) {
        int n = path.length();
        velocities[n - 1] = end_vel;
        for (int i0 = n - 2; i0 >= 0; --i0) {
            int i1 = i0 + 1;
            if (DEBUG)
                System.out.printf("BACK i %d\n", i0);
            double arclength = distances[i1] - distances[i0];
            if (Math.abs(arclength) < EPSILON) {
                // already handled this case
                break;
            }

            double maxVelocity = maxVelocity(path.getEntry(i0).point());
            if (DEBUG)
                System.out.printf("maxV i %d %f\n", i0, maxVelocity);

            double impliedAccel = Math100.accel(velocities[i0], velocities[i1], arclength);
            // Apply the decel constraint at the end of the segment since it is feasible.
            double maxDecelAtI1 = maxDecel(path.getEntry(i1).point(), velocities[i1]);
            if (impliedAccel < maxDecelAtI1) {
                velocities[i0] = Math100.v0(velocities[i1], maxDecelAtI1, arclength);
                if (DEBUG)
                    System.out.printf("1 adjust vi %f impliedA %f\n", velocities[i0], impliedAccel);
            }
            // This can produce an infeasible result at i0 so apply it again there.
            // This will be conservative, which is better than violating the constraint.
            impliedAccel = Math100.accel(velocities[i0], velocities[i1], arclength);
            double maxDecelAtI0 = maxDecel(path.getEntry(i0).point(), velocities[i0]);
            if (impliedAccel < maxDecelAtI0) {
                velocities[i0] = Math100.v0(velocities[i1], maxDecelAtI0, arclength);
                if (DEBUG)
                    System.out.printf("2 adjust vi %f impliedA %f\n", velocities[i0], impliedAccel);
            }

            if (Math.abs(maxVelocity) < velocities[i0]) {
                velocities[i0] = Math.signum(velocities[i0]) * maxVelocity;
                if (DEBUG)
                    System.out.println("fix v one more time");
            }

            if (DEBUG) {
                System.out.printf("BACK i0 %d vi0 %f vi1 %f max %f implied %f\n",
                        i0, velocities[i0], velocities[i1], maxDecelAtI1, impliedAccel);
            }
        }
    }

    /**
     * Returns the lowest (i.e. closest to zero) velocity constraint from the list
     * of constraints. Always positive or zero.
     */
    private double maxVelocity(PathSE2Point point) {
        double minVelocity = HIGH_V;
        for (TimingConstraint constraint : m_constraints) {
            minVelocity = Math.min(minVelocity, constraint.maxV(point));
        }
        return minVelocity;
    }

    /**
     * Returns the lowest (i.e. closest to zero) acceleration constraint from the
     * list of constraints. Always positive or zero.
     */
    private double maxAccel(PathSE2Point point, double velocity) {
        double minAccel = HIGH_ACCEL;
        for (TimingConstraint constraint : m_constraints) {
            minAccel = Math.min(minAccel, constraint.maxAccel(point, velocity));
        }
        return minAccel;
    }

    /**
     * Returns the highest (i.e. closest to zero) deceleration constraint from the
     * list of constraints. Always negative or zero.
     */
    private double maxDecel(PathSE2Point point, double velocity) {
        double maxDecel = -HIGH_ACCEL;
        for (TimingConstraint constraint : m_constraints) {
            maxDecel = Math.max(maxDecel, constraint.maxDecel(point, velocity));
        }
        return maxDecel;
    }

    private static double dt(
            double v0,
            double v1,
            double arcLength,
            double accel) {
        if (Math.abs(accel) > EPSILON) {
            // If accelerating, find the time to go from v0 to v1.
            return (v1 - v0) / accel;
        }
        if (Math.abs(v0) > EPSILON) {
            // If moving, find the time to go distance dq at speed v0.
            return arcLength / v0;
        }
        return 0;
    }
}