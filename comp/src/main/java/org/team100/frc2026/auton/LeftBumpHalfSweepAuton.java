package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.se2.commands.DriveWithTrajectoryFunction;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.TrajectorySE2;
import org.team100.lib.trajectory.TrajectorySE2Factory;
import org.team100.lib.trajectory.TrajectorySE2Planner;
import org.team100.lib.trajectory.constraint.TimingConstraint;
import org.team100.lib.trajectory.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.constraint.VelocityLimitRegionConstraint;
import org.team100.lib.trajectory.path.PathSE2Factory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** An example of a simple sequence */
public class LeftBumpHalfSweepAuton implements AnnotatedCommand {
    private final LoggerFactory log;
    private final ControllerSE2 controller;
    private final Machinery machinery;
    private final List<TimingConstraint> constraints;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;

    public LeftBumpHalfSweepAuton(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        this.controller = controller;
        this.machinery = machinery;
        constraints = new TimingConstraintFactory(kinodynamics).auto(log.type(this));
        // In meters/second
        double maxBumpVelocity = 1;
        List<TimingConstraint> new_constraints = new ArrayList<>(constraints);

        // create a new VelocityRegionContstraint `slow_bump_zone`
        VelocityLimitRegionConstraint slow_bump_zone = new VelocityLimitRegionConstraint(
                log.name("zone1"), BumpZones.BLUE_BUMP_LEFT, maxBumpVelocity);
        VelocityLimitRegionConstraint slow_bump_zone2 = new VelocityLimitRegionConstraint(
                log.name("zone2"), BumpZones.BLUE_BUMP_RIGHT, maxBumpVelocity);
        VelocityLimitRegionConstraint slow_bump_zone3 = new VelocityLimitRegionConstraint(
                log.name("zone3"), BumpZones.RED_BUMP_LEFT, maxBumpVelocity);
        VelocityLimitRegionConstraint slow_bump_zone4 = new VelocityLimitRegionConstraint(
                log.name("zone4"), BumpZones.RED_BUMP_RIGHT, maxBumpVelocity);
        new_constraints.add(slow_bump_zone);
        new_constraints.add(slow_bump_zone2);
        new_constraints.add(slow_bump_zone3);
        new_constraints.add(slow_bump_zone4);
        // constraints.add(slow_bump_zone);
        trajectoryFactory = new TrajectorySE2Factory(new_constraints);
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    @Override
    public String name() {
        return "Half Sweep Left Bump Auton";
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(AutonPositions.ABOVE_BALL_FIELD,
                        new DirectionSE2(1, 1, 0), 1));
        return planner.restToRest(waypoints);
    }

    TrajectorySE2 t2(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(0, -1, 0), 1),
                new WaypointSE2(AutonPositions.MIDDLE_BALL_FIELD,
                        new DirectionSE2(0, -1, 0), 1));
        return planner.restToRest(waypoints);
    }

    TrajectorySE2 t3(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(-1, 1, 0), 1),
                new WaypointSE2(StartingPositions.LEFT_BUMP,
                        new DirectionSE2(-1, 0, 0), 1),
                new WaypointSE2(AutonPositions.SHOOT_LEFT,
                        new DirectionSE2(-1, -1, 0), 1));
        return planner.restToRest(waypoints);
    }

    TrajectorySE2 t4(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(-1, -1, 0), 1),
                new WaypointSE2(AutonPositions.CLIMB_LEFT,
                        new DirectionSE2(-1, -1, 0), 1));
        return planner.restToRest(waypoints);
    }

    @Override
    public Command command() {
        DriveWithTrajectoryFunction IntakeSetUp = new DriveWithTrajectoryFunction(
                log.name("IntakeSetUp"), machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);
        DriveWithTrajectoryFunction IntakeBalls = new DriveWithTrajectoryFunction(
                log.name("IntakeBalls"), machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t2);
        DriveWithTrajectoryFunction ScoreSetUp = new DriveWithTrajectoryFunction(
                log.name("scoreSetup"), machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t3);
      //  DriveWithTrajectoryFunction ClimbSetUp = new DriveWithTrajectoryFunction(
               // log.name("ClimbSetup"), machinery.m_drive, controller,
               // machinery.m_trajectoryViz, this::t4);

        // Intake, score, climb.
        return sequence(
                Commands.print("foo"),
                parallel(
                        IntakeSetUp.until(IntakeSetUp::isDone),
                        // Assumed that the intake shouldn't deploy over the bump
                        waitSeconds(1).andThen(machinery.m_intakeExtend.goToExtendedPosition())),
                Commands.print("foo2"),
                waitSeconds(1),
                Commands.print("foo3"),
                parallel(
                        IntakeBalls,
                        machinery.m_intake.intake()).until(IntakeBalls::isDone),
                Commands.print("foo4"),
                // Without telling it to, the intake would only stop spinning
                // at the end of the auton. Without the timeout, the robot
                // would not continue the rest of the auton
                machinery.m_intake.stop().withTimeout(1),
                Commands.print("foo5"),
                waitSeconds(1),
                ScoreSetUp.until(ScoreSetUp::isDone),
                machinery.m_shooter.shooterFullspeed().withTimeout(1),
                waitSeconds(2),
                machinery.m_shooter.stop().withTimeout(1)

               // ClimbSetUp.until(ClimbSetUp::isDone)
               );
    }

    @Override
    public Pose2d start() {
        return StartingPositions.LEFT_BUMP;
    }

}
