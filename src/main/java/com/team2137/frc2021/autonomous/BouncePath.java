package com.team2137.frc2021.autonomous;

import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand.*;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import com.team2137.libs.TrajectoryUtility;
import com.team2137.libs.UnitsExtra;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;

public class BouncePath extends SequentialCommandGroup {
    public BouncePath(SwerveDrivetrain drivetrain, Intake intake) {
        var trajectory1 = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(2.85, 7.84,  Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(6.184, 7.887),
                        new Translation2d(8.296, 8.35)),
                new Pose2d(8.455, 10.25,  Rotation2d.fromDegrees(90)),
                drivetrain.getDefaultConstraint()
        );

        var setPoseCommand = new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(trajectory1.getInitialPose().getTranslation(), Rotation2d.fromDegrees(90))));

        var intake1Command = new SetIntakeCommand(intake, Intake.IntakeState.Deployed);
        var trajectory1Command = new TrajectoryFollowCommand(drivetrain, trajectory1, Rotation2d.fromDegrees(90));

        var firstGroup = new ParallelCommandGroup(new SequentialCommandGroup(new WaitCommand(1), intake1Command), trajectory1Command);

        var trajectory2 = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(8.455, 10.25,  Rotation2d.fromDegrees(270)),
                List.of(
                        new Translation2d(9.855, 4.552),
                        new Translation2d(11.707, 0.156),
                        new Translation2d(15.536, 0.052),
                        new Translation2d(15.912, 5.002)),
                new Pose2d(15.833, 8.669,  Rotation2d.fromDegrees(90)),
                drivetrain.getDefaultConstraint()
        );

        var intake2Command1 = new SetIntakeCommand(intake, Intake.IntakeState.Retracted);
        var intake2Command2 = new SetIntakeCommand(intake, Intake.IntakeState.Deployed);
        var trajectory2Command = new TrajectoryFollowCommand(drivetrain, trajectory2, Rotation2d.fromDegrees(110),
                new HeadingControlThreshold(UnitsExtra.feetToMeters(new Translation2d(12.5, 0)), UnitsExtra.feetToMeters(new Translation2d(12.5, 1)), Rotation2d.fromDegrees(90), ThresholdType.Static));

        var secondGroup = new ParallelCommandGroup(intake2Command1, trajectory2Command,
                new ParallelCommandGroup(new WaitCommand(trajectory2.getTotalTimeSeconds() - 0.5), intake2Command2));

        addCommands(setPoseCommand, firstGroup, secondGroup);
    }
}