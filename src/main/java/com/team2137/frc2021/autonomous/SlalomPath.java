package com.team2137.frc2021.autonomous;


import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import com.team2137.frc2021.util.CommandRunner;
import com.team2137.libs.TrajectoryUtility;
import com.team2137.libs.UnitsExtra;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.swing.plaf.basic.BasicComboBoxUI;
import java.util.List;

public class SlalomPath extends SequentialCommandGroup {
    public SlalomPath(SwerveDrivetrain drivetrain) {
        var trajectory = TrajectoryUtility.generateTrajectoryFeet(
                new Pose2d(3.16, 1.875,  Rotation2d.fromDegrees(20.394)),
                List.of(
                        new Translation2d(7.126, 3.047),
                        new Translation2d(8.395, 4.96),
                        new Translation2d(9.001, 7.388),
                        new Translation2d(12.977, 8.267),
                        new Translation2d(22.617, 8.662),
                        new Translation2d(24.421, 4.669),
                        new Translation2d(25.949, 0.468),
                        new Translation2d(29.678, 2.34),
                        new Translation2d(29.122, 7.497),
                        new Translation2d(23.945, 8.724),
                        new Translation2d(23.251, 4.482),
                        new Translation2d(20.256, 1.425),
                        new Translation2d(10.478, 1.757),
                        new Translation2d(8.157, 4.815)),
                new Pose2d(5.995, 8.537,  Rotation2d.fromDegrees(114.114)),
//                drivetrain.getDefaultConstraint()
                drivetrain.getDefaultConstraint().setEndVelocity(15)
        );

        SmartDashboard.putNumber("Slalom traj time", trajectory.getTotalTimeSeconds());

        var setPoseCommand = new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(trajectory.getInitialPose().getTranslation(), Rotation2d.fromDegrees(90))));

        var trajectoryCommand = new TrajectoryFollowCommand(drivetrain, trajectory, Rotation2d.fromDegrees(90),
                new TrajectoryFollowCommand.HeadingControlThreshold(UnitsExtra.feetToMeters(new Translation2d(20, 0)), UnitsExtra.feetToMeters(new Translation2d(20, 1)), Rotation2d.fromDegrees(90), TrajectoryFollowCommand.ThresholdType.Static),
                new TrajectoryFollowCommand.HeadingControlThreshold(UnitsExtra.feetToMeters(new Translation2d(15, 0)), UnitsExtra.feetToMeters(new Translation2d(15, 1)), Rotation2d.fromDegrees(120), TrajectoryFollowCommand.ThresholdType.Static));

        addCommands(setPoseCommand, trajectoryCommand);
    }
}