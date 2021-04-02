package com.team2137.frc2021.autonomous;

import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.commands.SetSpindexerCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.Spindexer;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import com.team2137.frc2021.util.CommandRunner;
import com.team2137.libs.TrajectoryUtility;
import com.team2137.libs.UnitsExtra;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class GalacticSearchBBlue extends ParallelCommandGroup {

    public GalacticSearchBBlue(SwerveDrivetrain drivetrain, Intake intake, Spindexer spindexer) {
        var startPose = new Pose2d(3.02, 9.08, Rotation2d.fromDegrees(0));

        var trajectory = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(1.5, 9.08, Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(13.25, 5.0),
                        new Translation2d(18.25, 11.5),
                        new Translation2d(23.25, 6)),
                new Pose2d(27, 4, Rotation2d.fromDegrees(0)),
                drivetrain.getDefaultConstraint()
        );

        var setPoseCommand = new InstantCommand(() -> drivetrain.resetOdometry(UnitsExtra.feetToMeters(startPose)));

        var intakeCommand = new SetIntakeCommand(intake, Intake.IntakeState.Running);
        var spindexerCommand = new SetSpindexerCommand(spindexer,1);
        var trajectoryCommand = new TrajectoryFollowCommand(drivetrain, trajectory, Rotation2d.fromDegrees(0));

        addCommands(new ParallelCommandGroup(setPoseCommand, intakeCommand, spindexerCommand), trajectoryCommand);
    }
}
