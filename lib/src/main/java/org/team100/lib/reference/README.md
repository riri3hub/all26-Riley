# lib.reference

Classes here supply "references" (controller setpoints).

There are two dimensionalities:

* R1: one-dimensional, used for single-DOF mechanisms.
* SE(2): planar, i.e. `Pose2d`, used for drivetrain and planar mechanism.

The reference sources can wrap two different suppliers:

* Profiles
* Trajectories

The reason for this layer is to decouple the controller code from
the reference source, so the same controller can be used for, say, a profile
or a trajectory.

An important trait of Team 100 reference generators is that they produce two
setpoints: one for the "current time step" and one for the "next time step."
The reason to track these separately is that they are used for different
purposes:

* The "current" setpoint represents where the mechanism should be at the
current instant in time (defined by `Takt`), and so it should be compared
with the current measurement (as obtained by `Cache`), to derive the
the controller error, and form the basis of __feedback control.__  For example,
the simplest feedback control law would be to simply feed the (scaled) error
into the output: this is called __proportional feedback__.
* The "next" setpoint represents where the mechanism should be at the next
`Takt` instant.  This is useful to guide __feedforward control__.  Since
most of our actuators are velocity servos, most of our feedforwards are
simply "next setpoint velocity" passthroughs.