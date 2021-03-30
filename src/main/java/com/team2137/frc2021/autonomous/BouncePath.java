package com.team2137.frc2021.autonomous;


import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import com.team2137.frc2021.util.CommandRunner;
import com.team2137.libs.TrajectoryUtility;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class BouncePath {
    public BouncePath(RobotContainer robot) {
        var startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        var setPoseCommand = new InstantCommand(() -> robot.drivetrain.resetOdometry(startPose));

        var trajectory = TrajectoryUtility.generateTrajectoryFeet(
                startPose,
                List.of(new Translation2d(7.5, 4)),
                new Pose2d(new Translation2d(15, 0), Rotation2d.fromDegrees(0)),
                robot.drivetrain.getDefaultConstraint()
        );

        var trajectoryCommand = new TrajectoryFollowCommand(robot.drivetrain, trajectory, Rotation2d.fromDegrees(0));

        CommandRunner.executeCommandSequence(setPoseCommand, trajectoryCommand);
    }
}