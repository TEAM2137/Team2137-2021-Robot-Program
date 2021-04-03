package com.team2137.frc2021.autonomous;


import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import com.team2137.frc2021.util.CommandRunner;
import com.team2137.libs.TrajectoryUtility;
import com.team2137.libs.UnitsExtra;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class BarrelRacing extends SequentialCommandGroup {
    public BarrelRacing(SwerveDrivetrain drivetrain) {
        var startPose = new Pose2d(2.85, 7.84, Rotation2d.fromDegrees(0));

        var trajectory = TrajectoryUtility.generateTrajectoryFeet(
                startPose,
                List.of(
                        new Translation2d(11.885, 7.88),
                        new Translation2d(16.826, 4.699),
                        new Translation2d(12.449, 1.674),
                        new Translation2d(7.889, 4.945),
                        new Translation2d(12.522, 7.742),
                        new Translation2d(22.425, 8.765),
                        new Translation2d(24.503, 11.513),
                        new Translation2d(21.187, 13.872),
                        new Translation2d(15.724, 11.453),
                        new Translation2d(21.824, 3.332),
                        new Translation2d(27.596, 2.564),
                        new Translation2d(29.36, 6.936),
                        new Translation2d(25.587, 8.196),
                        new Translation2d(18.074, 8.101),
                        new Translation2d(9.641, 9.258)),
                new Pose2d(1.204, 10.252,  Rotation2d.fromDegrees(180)),
//                drivetrain.getDefaultConstraint().addConstraint(new CentripetalAccelerationConstraint(Units.feetToMeters(6)))
                drivetrain.getDefaultConstraint().setEndVelocity(Units.feetToMeters(13))
//                drivetrain.getDefaultConstraint()
        );

        SmartDashboard.putNumber("Barrel Racing Traj Time", trajectory.getTotalTimeSeconds());

        var setPoseCommand = new InstantCommand(() -> drivetrain.resetOdometry(UnitsExtra.feetToMeters(startPose)));

        var trajectoryCommand = new TrajectoryFollowCommand(drivetrain, trajectory, Rotation2d.fromDegrees(0));

        addCommands(setPoseCommand, trajectoryCommand);
    }
}