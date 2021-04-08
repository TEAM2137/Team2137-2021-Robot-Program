package com.team2137.frc2021.autonomous;

import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.commands.SetSpindexerCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand.*;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.Spindexer;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import com.team2137.frc2021.util.CommandRunner;
import com.team2137.libs.TrajectoryUtility;
import com.team2137.libs.UnitsExtra;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class GalacticSearchBBlue extends SequentialCommandGroup {

    Timer timer;

    public GalacticSearchBBlue(SwerveDrivetrain drivetrain, Intake intake, Spindexer spindexer) {
        timer = new Timer();

        var startPose = new Pose2d(3.02, 9.08, Rotation2d.fromDegrees(0));

        var trajectory = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(1.5, 9.08, Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(13.25, 5.5),
                        new Translation2d(20.0, 10.0),
                        new Translation2d(23, 5.5)),
                new Pose2d(31, 5, Rotation2d.fromDegrees(-20)),
                drivetrain.getDefaultConstraint().setEndVelocity(Units.feetToMeters(15))
        );

        var setPoseCommand = new InstantCommand(() -> drivetrain.resetOdometry(UnitsExtra.feetToMeters(startPose)));

        var intakeCommand = new SetIntakeCommand(intake, Intake.IntakeState.Running);
        var spindexerCommand = new SetSpindexerCommand(spindexer,1);
        var trajectoryCommand = new TrajectoryFollowCommand(drivetrain, trajectory, Rotation2d.fromDegrees(-20),
                new HeadingControlThreshold(UnitsExtra.feetToMeters(new Translation2d(14, 6)), UnitsExtra.feetToMeters(new Translation2d(17, 3)),
                        Rotation2d.fromDegrees(40), ThresholdType.Static),
                new HeadingControlThreshold(UnitsExtra.feetToMeters(new Translation2d(17.5, 0)), UnitsExtra.feetToMeters(new Translation2d(17.5, 1)),
                        Rotation2d.fromDegrees(-42), ThresholdType.Static));

        var timerStartCommand = new InstantCommand(() -> {
            timer.reset();
            timer.start();
        });

        var timerEndCommand = new InstantCommand(() -> {
            timer.stop();
            SmartDashboard.putNumber("Time", timer.get());
        });

        addCommands(new ParallelCommandGroup(setPoseCommand, intakeCommand, spindexerCommand, timerStartCommand), trajectoryCommand, timerEndCommand);
    }
}
