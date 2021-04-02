package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.util.CommandRunner;
import com.team2137.libs.TrajectoryUtility;
import com.team2137.libs.UnitsExtra;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.ArrayList;
import java.util.List;

public class Autonomous extends RobotContainer implements OpMode {

    @Override
    public void init() {
//        intake.setIntakeState(Intake.IntakeState.Running);
//        spindexer.setPower(1);
//
//        var startPose = new Pose2d(3.02, 9.08, Rotation2d.fromDegrees(0));
//
//        var trajectory = TrajectoryUtility.generateTrajectoryFeet(
//                new Pose2d(1.5, 9.08, Rotation2d.fromDegrees(0)),
//                List.of(
//                        new Translation2d(13.25, 4.25),
//                        new Translation2d(18.25, 10.75),
//                        new Translation2d(23.25, 5)),
//                new Pose2d(27, 4, Rotation2d.fromDegrees(0)),
//                drivetrain.getDefaultConstraint()
//        );
//
//        drivetrain.resetOdometry(UnitsExtra.feetToMeters(startPose));
//
//        var trajectoryCommand = new TrajectoryFollowCommand(drivetrain, trajectory, Rotation2d.fromDegrees(0));
//
//        CommandRunner.executeCommand(trajectoryCommand);

        CommandRunner.executeCommand(autoSelector.getSelected());
    }

    @Override
    public void periodic() {

    }

    @Override
    public void end() {

    }
}
