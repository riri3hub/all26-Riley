package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.List;
import java.util.ArrayList;

import org.team100.frc2026.Intake;
import org.team100.frc2026.auton.BumpZones;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example of a simple sequence */
public class Auton1 implements AnnotatedCommand {
    private final LoggerFactory log;
    private final ControllerSE2 controller;
    private final Machinery machinery;
    private final List<TimingConstraint> constraints;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;

    public Auton1(
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
         
        // create a new VelocityRegionContstraint `slow_bu  mp_zone`
        VelocityLimitRegionConstraint slow_bump_zone = new VelocityLimitRegionConstraint(log, BumpZones.BLUE_BUMP_LEFT, maxBumpVelocity);
        new_constraints.add(slow_bump_zone);
        // constraints.add(slow_bump_zone);
        trajectoryFactory = new TrajectorySE2Factory(new_constraints);
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    @Override
    public String name() {
        return "Auton 1";
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
        DriveWithTrajectoryFunction n1 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);
        DriveWithTrajectoryFunction n2 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t2);
        DriveWithTrajectoryFunction n3 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t3);
        DriveWithTrajectoryFunction n4 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t4);
        return sequence(
                n1.until(n1::isDone),
                waitSeconds(1),
                parallel(
                    n2,
                   machinery.m_intake.intake()
                ).until(n2::isDone),
                waitSeconds(1),
                n3.until(n3::isDone),
                waitSeconds(1),
                n4.until(n4::isDone));
    }

    @Override
    public Pose2d start() {
        return StartingPositions.LEFT_BUMP;
    }

}
