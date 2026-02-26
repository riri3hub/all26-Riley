package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

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
import org.team100.lib.trajectory.path.PathSE2Factory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class BClimberAutonLeft implements AnnotatedCommand {
    private final LoggerFactory log;
    private final ControllerSE2 controller;
    private final Machinery machinery;
    private final List<TimingConstraint> constraints;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;

    public BClimberAutonLeft(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        this.controller = controller;
        this.machinery = machinery;
        constraints = new TimingConstraintFactory(kinodynamics).auto(log.type(this));
        trajectoryFactory = new TrajectorySE2Factory(constraints);
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    @Override
    public String name() {
        return "BClimber Auton Left";
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(-1, 0, 0), 1),
                new WaypointSE2(AutonPositions.CLIMB_LEFT,
                        new DirectionSE2(0, -1, 0), 1));
        return planner.restToRest(waypoints);
    }

    @Override
    public Command command() {
        DriveWithTrajectoryFunction n1 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);
        return sequence(
                n1.until(n1::isDone).withTimeout(1),
                machinery.m_ClimberExtension.setPosition(),
                waitSeconds(1).andThen(machinery.m_Climber.setClimb1()));
    }

    @Override
    public Pose2d start() {
        return StartingPositions.LEFT_BUMP;
    }

}
