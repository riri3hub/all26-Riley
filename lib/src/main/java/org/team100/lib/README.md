# lib

This package contains library code we can use every year.

If you're trying to learn about the library, you should start with the README in each package.

__Packages__

* [camera](camera/README.md)
  * Configuration for cameras used for localization and targeting.
* [coherence](coherence/README.md)
  * A discrete clock and cache intended to manage observations (so they all represent the same instant) and anything else that depends on time (e.g. steps along a profile).
* [commands](commands/README.md)
  * High level navigation commands, using manual control, profiles, and trajectories.
* [config](config/README.md)
  * The most interesting config is `Identity`, which is an enum of RoboRIO
  serial numbers.  We use this identity to differentiate robot hardware within
  a single code-base.
* [controller](controller/README.md)
  * Feedback controllers for one dimension and three independent dimensions.  We use the latter for 2d-with-heading poses.  Some controllers do feedforward, some do feedback, some do both.
* [dynamics](dynamics/README.md)
  * Models mass, inertia, and forces, including centrifugal and coriolis forces, of planar mechanisms.
* [examples](examples/README.md) 
  * A few illustrations of using library features.
* [experiments](experiments/README.md) 
  * Switches that control optional features using dashboard selectors.
* [field](field/README.md)
  * Constants representing significant coordinates on the playing field.
* [framework](framework/README.md)
  * Team 100 versions of WPI code, with better logging.
* [geometry](geometry/README.md)
  * Utility code for geometric computations, particularly useful in 3d, and to keep "robot relative" and "field relative" straight.
* [hid](hid/README.md) 
  * Human interfaces, mostly simple wrappers.
* [indicator](indicator/README.md)
  * Ways to get the drive team's attention.
* [kinematics](kinematics/README.md)
  * Models the geometry of motion, for multi-link planar mechanisms.
* [localization](localization/README.md)
  * We use camera sightings of canonical AprilTags, combined with odometry and gyro input, for full-field localization, used in navigation and pose-aware commands, e.g. maintaining a shooter aim point.
* [logging](logging/README.md) 
  * Loggers are passed through constructors, so the tree in Network Tables mirrors the construction graph, as a starting point.
* [mechanism](mechanism/README.md)
  * A Team 100 "mechanism" is a wrapper around a motor and encoder, with some knowledge of gearing.
* [motor](motor/README.md) 
  * Wrappers for all the motors we use (including simulated motors), so they can be handled uniformly by the mechanism classes above.
* [music](music/README.md)
  * Wrapper for CTRE MusicTone.
* [network](network/README.md)
  * Reads stuff off the network, sent by coprocessors (e.g. cameras).
* [optimization](optimization/README.md)
  * Solvers used in applications like inverse kinematics, notably an implementation of Newton's method
for finding the zero of a function.
* [profile](profile/README.md)
  * Constrained motion in 1d and 2d-with-heading.  These should be used for all motion where a trajectory would be too expensive to compute on the fly.
* [reference](reference/README.md)
  * Team 100 reference generators.  These use any sort of reference source, e.g. profile or trajectory, and produce "current" and "next" setpoints.  The reason for these classes is to make sure the time-alignment of observation and control are done correctly.  They use the `coherence` machinery mentioned above.
* [sensor](sensor/README.md)
  * Wrappers for measurement of position (via absolute rotary encoders), distance (via TOF sensors like the LaserCAN), orientation (via the Redux gyro), and the state of switches or photo-interruptors.
* [servo](servo/README.md)
  * A Team 100 "servo" combines a "mechanism" above with closed-loop control.  Give it a goal, and it goes there.
* [state](state/README.md)
  * Classes to represent position, velocity, etc, in various dimensions.
* [subsystems](subsystems/README.md)
  * Subsystem code, and associated commands, that you can use in your robot.
* [targeting](targeting/README.md)
  * A fundamental robot navigation task is to identify targets to drive to.  The `lib.targeting` package turns camera observations into field-relative targets.
* [trajectory](trajectory/README.md)
  * A trajectory is a path based on splines, with a precalculated schedule meeting timing constraints.  Trajectories are good for paths that require curves around known obstacles.  Simple trajectories are not *that* time-consuming to create, so can be used on-the-fly in some cases.
* [tuning](tuning/README.md)
  * Adjust constants on the fly.
* [util](util/README.md)
  * Small things that didn't fit anywhere else
* [visualization](visualization/README.md)
  * Glass widgets useful for simulation -- also work in comp, but we never look at them.

Other notable topics about the library:

__Simulation__

Our approach to simulation is completely different from the WPILib approach,
in two important ways. First, WPILib attempts to simulate realistic mechanism
physics: we do not.  Second, WPILib attempts to involve vendors (e.g. REV) in
the simulation of their hardware: we do not.

Our approach to simulation is to use simulated motors that respond instantly
to their inputs.  The expectation is that inputs will be _feasible_.  Team
100 simulation is not useful to understand mechanism physics, it is only
useful for higher-level concerns, like designing paths to follow.

Our simulated motor implementations have nothing to do with the vendor
implementations; we use `Identity` in the constructor tree to select the
simulated implementation (for the `BLANK` identity).

## Design

Contributions to `lib` are welcome!  Here are some things to keep in mind:

* Tests
  * As much as reasonable, lib code should be covered with tests, both to
support any claim of correctness, but also to __demonstrate usage.__
* Simplicity
  * As Richard Gabriel [once said](https://en.wikipedia.org/wiki/Worse_is_better),
"Simplicity is the most important consideration in a design."  The code here
is primarily a teaching tool for high-school students, so they should
be able to read and understand every bit of it.
* Consistency
  * Use a consistent style.  I don't like Hungarian-like thing that WPI does
(using the leading "m_"), but they do it, and thus, so do we.
* Smallness
  * Classes should be short, and only do one thing.
* Immutability
  * As much as possible, use immutability to avoid complexity.
