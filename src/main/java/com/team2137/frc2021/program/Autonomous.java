package com.team2137.frc2021.program;

import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.libs.TrajectoryUtility;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;

public class Autonomous extends RobotContainer implements OpMode {

    @Override
    public void init() {
        Trajectory trajectory;
        TrajectoryFollowCommand followCommand;

        ArrayList<Translation2d> interior = new ArrayList<>();
        interior.add(new Translation2d(7.5, 2));
        trajectory = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                interior,
                new Pose2d(new Translation2d(15, 0), Rotation2d.fromDegrees(0)),
                drivetrain.getDefaultConstraint()
        );

        ArrayList<Translation2d> interior2 = new ArrayList<>();
        interior2.add(new Translation2d(4, 2));
        interior2.add(new Translation2d(8, -2));
        Trajectory traj2 = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                interior2,
                new Pose2d(new Translation2d(12, 0), Rotation2d.fromDegrees(0)),
                drivetrain.getDefaultConstraint()
        );


//        followCommand = new TrajectoryFollowCommand(drivetrain, trajectory, Rotation2d.fromDegrees(90));
        TrajectoryFollowCommand.HeadingControlThreshold thresh = new TrajectoryFollowCommand.HeadingControlThreshold(-10, 22.86, Rotation2d.fromDegrees(0));
        var headingThresholds = new TrajectoryFollowCommand.HeadingControlThreshold[]{thresh};
//        followCommand = new TrajectoryFollowCommand(drivetrain, trajectory, Rotation2d.fromDegrees(90), headingThresholds);

        drivetrain.selfTargetAllModuleAngles();

        followCommand = new TrajectoryFollowCommand(drivetrain, traj2, Rotation2d.fromDegrees(180));
        CommandScheduler.getInstance().schedule(followCommand);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void end() {

    }
}
