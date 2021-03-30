package com.team2137.frc2021.autonomous;

import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.commands.SetSpindexerCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.Spindexer;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import com.team2137.libs.TrajectoryUtility;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class GalacticSearchABlue extends ParallelCommandGroup {

    public GalacticSearchABlue(SwerveDrivetrain drivetrain, Intake intake, Spindexer spindexer) {
        var trajectory = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(1, 7, Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(7.7, 6.7),
                        new Translation2d(12.2, 5.6),
                        new Translation2d(15.5, 12.6)),
                new Pose2d(28.7, 12.8, Rotation2d.fromDegrees(0)),
                drivetrain.getDefaultConstraint()
        );

        var intakeCommand = new SetIntakeCommand(intake, Intake.IntakeState.Running);
        var spindexerCommand = new SetSpindexerCommand(spindexer,1);
        var trajectoryCommand = new TrajectoryFollowCommand(drivetrain, trajectory, Rotation2d.fromDegrees(0));

        addCommands(intakeCommand, spindexerCommand, trajectoryCommand);
    }
}
