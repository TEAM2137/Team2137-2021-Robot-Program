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
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;
import java.util.Set;

public class BouncePath extends SequentialCommandGroup {
    public BouncePath(SwerveDrivetrain drivetrain, Intake intake) {
        var trajectory1 = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(2.85, 7.84,  Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(5.684, 7.887),
                        new Translation2d(7.996, 8.35)),
                new Pose2d(8.255, 10.25,  Rotation2d.fromDegrees(90)),
                drivetrain.getDefaultConstraint()
        );

        var setPoseCommand = new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(trajectory1.getInitialPose().getTranslation(), Rotation2d.fromDegrees(90))));

        var intake1Command = new SetIntakeCommand(intake, Intake.IntakeState.Deployed);
        var trajectory1Command = new TrajectoryFollowCommand(drivetrain, trajectory1, Rotation2d.fromDegrees(90));

        var firstGroup = new ParallelCommandGroup(new SequentialCommandGroup(new WaitCommand(1), intake1Command), trajectory1Command);

        var trajectory2 = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(8.255, 10.25,  Rotation2d.fromDegrees(270)),
                List.of(
                        new Translation2d(9.253, 7.211),
                        new Translation2d(11.992, 3.868),
                        new Translation2d(12.357, 0.229),
                        new Translation2d(16.536, 0.195),
                        new Translation2d(16.587, 4.328)),
                new Pose2d(16.344, 9.701,  Rotation2d.fromDegrees(90)),
                drivetrain.getDefaultConstraint()
        );

        //new Pose2d(8.455, 10.25,  Rotation2d.fromDegrees(270)),
        //List.of(
        //   new Translation2d(10.767, 4.76),
        //   new Translation2d(12.481, 0.863),
        //   new Translation2d(16.012, 0.821),
        //   new Translation2d(16.606, 4.94)),
        //new Pose2d(16.547, 9.293,  Rotation2d.fromDegrees(90)),

        var intake2Command1 = new SetIntakeCommand(intake, Intake.IntakeState.Retracted);
        var intake2Command2 = new SetIntakeCommand(intake, Intake.IntakeState.Deployed);
        var trajectory2Command = new TrajectoryFollowCommand(drivetrain, trajectory2, Rotation2d.fromDegrees(110),
                new HeadingControlThreshold(UnitsExtra.feetToMeters(new Translation2d(10.5, 0)), UnitsExtra.feetToMeters(new Translation2d(10.5, 1)), Rotation2d.fromDegrees(90), ThresholdType.Static));

//        var secondGroup = new ParallelCommandGroup(intake2Command1, trajectory2Command,
//                new ParallelCommandGroup(new WaitCommand(3.0), intake2Command2));

        var secondGroup = new ParallelCommandGroup(intake2Command1, trajectory2Command,
                new SequentialCommandGroup(new WaitCommand(3.0), intake2Command2));

        var trajectory3 = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(16.344, 9.701,  Rotation2d.fromDegrees(270)),
                List.of(
                        new Translation2d(16.666, 3.608),
                        new Translation2d(17.318, 0),
                        new Translation2d(20.624, 0),
                        new Translation2d(23.367, 0.5),
                        new Translation2d(24.661, 4.378)),
                new Pose2d(24.793, 9.423,  Rotation2d.fromDegrees(90)),
                drivetrain.getDefaultConstraint()
        );

        var intake3Command1 = new SetIntakeCommand(intake, Intake.IntakeState.Retracted);
        var intake3Command2 = new SetIntakeCommand(intake, Intake.IntakeState.Deployed);
        var trajectory3Command = new TrajectoryFollowCommand(drivetrain, trajectory3, Rotation2d.fromDegrees(90));

        var thirdGroup = new ParallelCommandGroup(intake3Command1, trajectory3Command,
                new SequentialCommandGroup(new WaitCommand(3.0), intake3Command2));

        var trajectory4 = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(24.793, 9.423,  Rotation2d.fromDegrees(-65.54)),
                List.of(),
                new Pose2d(27.607, 5.087,  Rotation2d.fromDegrees(0)),
                drivetrain.getDefaultConstraint().setEndVelocity(Units.feetToMeters(15))
        );

        var intake4Command = new SetIntakeCommand(intake, Intake.IntakeState.Retracted);
        var trajectory4Command = new TrajectoryFollowCommand(drivetrain, trajectory4, Rotation2d.fromDegrees(120));

        var fourthGroup = new ParallelCommandGroup(intake4Command, trajectory4Command);

        addCommands(setPoseCommand, firstGroup, secondGroup, thirdGroup, fourthGroup);
//        addCommands(setPoseCommand, firstGroup, secondGroup);
    }
}