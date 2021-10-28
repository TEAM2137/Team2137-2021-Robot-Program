package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.LimeLight;
import com.team2137.frc2021.util.CommandRunner;
import com.team2137.libs.TrajectoryUtility;
import com.team2137.libs.UnitsExtra;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.ArrayList;
import java.util.List;

public class Autonomous extends RobotContainer implements OpMode {

    Timer timer = new Timer();

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

//        CommandRunner.executeCommand(autoSelector.getSelected());

        timer.reset();
        timer.start();

        drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
    }

    @Override
    public void periodic() {
        double thetaPower = -shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX) * 1.4;

        double drivePower;

        if (timer.hasElapsed(10)) {
            shooter.idleFlyWheel();
            drivePower = 0.2;

            spindexer.setPower(0);
        } else if (timer.hasElapsed(13)) {
            shooter.idleFlyWheel();
            drivePower = 0;
            spindexer.setPower(0);
        } else {
            shooter.setShooterPosisition(10);
            drivePower = 0;

            if (shooter.isFlywheelAtTarget(100)) {
                spindexer.setPower(1);
            } else {
                spindexer.setPower(0);
            }
        }

        drivetrain.driveTranslationRotationRaw(ChassisSpeeds.fromFieldRelativeSpeeds(drivePower, 0, thetaPower, drivetrain.getRobotAngle()));


    }

    @Override
    public void end() {
        drivetrain.setAllModuleDriveRawPower(0);

    }
}
