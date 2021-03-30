package com.team2137.frc2021.autonomous;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.commands.SetSpindexerCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.Spindexer;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import com.team2137.frc2021.util.CommandRunner;
import com.team2137.libs.TrajectoryUtility;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class GalacticSearchARed {
    public GalacticSearchARed(RobotContainer robot) {
        var trajectory = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(1, 7, Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(7.7, 6.7),
                        new Translation2d(12.2, 5.6),
                        new Translation2d(15.5, 12.6)),
                new Pose2d(28.7, 12.8, Rotation2d.fromDegrees(0)),
                robot.drivetrain.getDefaultConstraint()
        );

        var intakeCommand = new SetIntakeCommand(robot.intake, Intake.IntakeState.Running);
        var spindexerCommand = new SetSpindexerCommand(robot.spindexer,1);
        var trajectoryCommand = new TrajectoryFollowCommand(robot.drivetrain, trajectory, Rotation2d.fromDegrees(0));

        CommandRunner.executeCommandGroup(intakeCommand, spindexerCommand, trajectoryCommand);
    }
}
