# Intercept

How to hit a moving target from a moving shooter platform.

In the global reference frame, the situation looks like this:

<img src="global_diagram.png" width=300 />

The robot position over time is $R_G(t) = R_{0_G} + v_{R_G} t$. 

The target position over time is $T_G(t) = T_{0_G} + v_{T_G} t$.

The math is easier if we use the robot-local reference frame.
Note this frame does not respect robot *rotation*, only robot *motion*.

<img src="local_diagram.png" width=400 />

Our task is to determine $\theta$, the shooter azimuth.  This angle
is a field-relative angle (i.e. relative to the global x axis),
since the "robot reference frame" here does not affect rotation.

The robot's position is fixed at the origin.

The target's initial position is $T_0 = T_{0_G} - R_{0_G}$.

The target velocity in the robot frame is $v_T = (v_{T_G} - v_{R_G})$

The target position over time in the robot frame is $T(t) = T_0 + v_T t$.

The projectile muzzle speed is always the same, $s_m$.

We don't know what the intersection position is, but we do know
its distance from the origin: $s_m t$.  When intersection occurs at $t$, the
target should be that far from the origin:

```math
s_m t = \left\| T_0 + v_T t \right\|
```

Square both sides:

```math
s_m^2 t^2 = \left\| T_0 + v_T t \right\|^2
```

The squared magnitude is the self-dot-product, so expanding:

```math
s_m^2 t^2 =  T_0 \cdot T_0 + 2(T_0 \cdot v_T) t + (v_T \cdot v_T) t^2
```

This is quadratic in $t$:

```math
(v_T \cdot v_T - s_m^2) t^2  + 2(T_0 \cdot v_T) t +   T_0 \cdot T_0 = 0
```

There are three possible solutions:

* __Two solutions:__ target can be reached on approach, or later while receding.  One of the solutions may be negative, which means it is too late to achieve the approach solution.  If there's more than one positive solution, we generally want the
smaller one.
* __One solution:__ target can just barely be reached.
* __No solution:__ target is moving away faster than the projectile can catch it.



Once we have a solution for $t$, we can use it to find $I$ by
following the target path:

```math
I = T_0 + v_r t
```

And finally use the $atan2$ function to get the angle we want,
using the components of $I$.

```math
\theta = atan2(I_y, I_x)
```

## Implementation

There is a prototype static method `Intercept.intercept()` suitable
for this work:

```java
    /**
     * Find a turret rotation which will intercept the target. If more than one
     * solution is possible, choose the sooner one. If no solution is possible,
     * return Optional.empty().
     * 
     * @param robotPosition  field-relative robot position, meters
     * @param robotVelocity  field-relative robot velocity, meters/sec
     * @param targetPosition field-relative target position, meters
     * @param targetVelocity field-relative target velocity, meters/sec
     * @param muzzleSpeed    speed of the projectile, meters/sec
     * @return field-relative firing solution azimuth
     */
    public static Optional<Rotation2d> intercept(
            Translation2d robotPosition,
            VelocityR2 robotVelocity,
            Translation2d targetPosition,
            VelocityR2 targetVelocity,
            double muzzleSpeed) {

        return Optional.empty();
    }

```

There are a few test cases you can do in your head in `InterceptTest`.
You should add many more of these simple cases, including some that
require a pencil to work out.

There are some key library classes to use here:

* `VelocityR2` is a good representation for the 2d
velocities, and has a convenience method, `dot()` for the dot
product with other velocities and also with translations.
* `GeometryUtil.dot()` is also handy for the dot product between
two translations.
* `Math100.solveQuadratic()` will return a list of zero, one, or two solutions to the quadratic equation defined above.


## Refinements

Once the initial implementation is done, it would be good to go 
back and add some refinements:

__Offset__

The shooter may not be at the robot position; it could be offset
a little.

__Delay__

The analysis above assumes that there is no delay in shooter actuation.
This assumption is false: every mechanism involves delay.  To
correct for delay, use the robot and target velocities to determine
robot and target positions after the delay period has elapsed.

__Elevation__

In addition to solving for azimuth, we should also solve for elevation.
This is a more complex analysis, since the projectile path is definitely
not a straight line (as it would be without any forces at all),
but it's also not a parabola (as it would be with only gravity),
it's a complex shape also affected by interaction with the air.
In the past, we've handled elevation with a lookup table.

__Muzzle speed is not constant__

The speed out of the shooter is constant, but the 2d projection of
that speed depends on elevation.  (Imagine an elevation of 85 degrees:
the projectile moves fast up and down but very slowly across the floor.)
To address this issue, we might adjust the real-world shooter speed so
that the resulting 2d speed really *is* constant, though that would
create other issues (e.g. very high arcs for long shots).  Another
option might be to replace the constant $s_m$ term above with something
that involves $t$ (at $I$), e.g. $s_m = s_0 - s_1t$, speed is slower when
trying to hit something far away (at a higher elevation).  Obviously
this linear approximation is only valid for small changes in elevation.The resulting
equation would be quartic, so it could be solved using one of the
solvers in `lib.optimization`, e.g. `Bisection1d` or `NewtonsMethod1d`.

# Iterative Solution

An alternative to the post-hoc elevation lookup and muzzle speed approximation above
would be to cast the problem as a two-dimensional optimization, solving
both azimuth and elevation to minimize the intercept error.

With this method, instead of solving directly for $t$, you would use an iterative
solver that guesses values for azimuth ($\theta$) and elevation ($\phi$).

The function to be minimized would use a lookup table from experiment or precomputation
to determine the range and time of arrival, and then, using the azimuth, find the
cartesian location.  Then compute the distance ("error") between that location and the
target at the same time.  The solver would use that distance to guide its guesses.

A useful lookup table for elevation would be `InterpolatingTreeMap`.

There are a few options in `lib.optimization` for the optimizer, e.g.
`GradientDescent` or `NewtonsMethod`.

A good choice for the initial guesses would be pointing at the target's
current location using the looked-up elevation for that location.

# Iteration on TOF

Another iterative approach evolves the target location with a given TOF (starting with zero),
and aims directly at that, until the TOF of the solution matches.  See `TimeOfFlightRecursion`.

## Computing range and time of flight

There is some code in `Drag` that describes the forces on a projectile,
and some code in `Range` that uses the `Drag` model to precompute the range
and TOF for various elevations.


## Resources
Some resources about this problem.

* https://indyandyjones.wordpress.com/2010/04/08/intercepting-a-target-with-projectile/
* https://playtechs.blogspot.com/2007/04/aiming-at-moving-target.html
* https://stackoverflow.com/questions/17204513/how-to-find-the-interception-coordinates-of-a-moving-target-in-3d-space
* [Geogebra global diagram](https://www.geogebra.org/m/hextbjj2)
* [Geogebra local diagram](https://www.geogebra.org/m/guchqzvs)
* [Trajectories in air](https://en.wikipedia.org/wiki/Projectile_motion#Trajectory_in_air)
* [Rangekeeper](https://en.wikipedia.org/wiki/Rangekeeper)
* [Rangekeeping math](https://en.wikipedia.org/wiki/Mathematical_discussion_of_rangekeeping)