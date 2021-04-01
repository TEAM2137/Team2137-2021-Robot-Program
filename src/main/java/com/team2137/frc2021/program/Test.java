package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.subsystems.Spindexer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

public class Test extends RobotContainer implements OpMode {
    private boolean intakeButtonPreviouslyPressed = false;
    private boolean intakePreviouslyDeployed = false;

//    ProfiledPIDController thetaController;
//
//    PIDController xController;
//    PIDController yController;

    ProfiledPIDController thetaController;

    Rotation2d target;

    @Override
    public void init() {
//        shooter.zeroHoodAngle();

//        Compressor c = new Compressor();

//        drivetrain.setAllModuleRotations(new Rotation2d(0));
//        c.stop();

//        thetaController = new ProfiledPIDController(Constants.Drivetrain.teleopThetaPIDConstants.getP(),
//                Constants.Drivetrain.teleopThetaPIDConstants.getI(),
//                Constants.Drivetrain.teleopThetaPIDConstants.getD(),
//                Constants.Drivetrain.teleopThetaPIDConstraints);
//
//        thetaController.enableContinuousInput(-Math.PI, Math.PI);
//
//        xController = new PIDController(Constants.Drivetrain.purePIDTranslationConstants.getP(),
//                Constants.Drivetrain.purePIDTranslationConstants.getI(),
//                Constants.Drivetrain.purePIDTranslationConstants.getD());
//
//        yController = new PIDController(Constants.Drivetrain.purePIDTranslationConstants.getP(),
//                Constants.Drivetrain.purePIDTranslationConstants.getI(),
//                Constants.Drivetrain.purePIDTranslationConstants.getD());

//        spindexer.setPower(1);
//        spindexer.setBallStop(Spindexer.BallStopState.Disabled);
//        shooter.setPreRollerPower(1);
//        shooter.zeroHoodAngle();
//        shooter.setHoodAngle(31.5);

//        SmartDashboard.putNumber("Hood Angle", 10.0);
//        SmartDashboard.putNumber("Flywheel Velocity Goal", 5400.0);

        target = drivetrain.getRobotAngle().plus(Rotation2d.fromDegrees(180));

        thetaController = new ProfiledPIDController(Constants.Drivetrain.autoThetaPIDConstants.getP(), Constants.Drivetrain.autoThetaPIDConstants.getI(), Constants.Drivetrain.autoThetaPIDConstants.getD(), Constants.Drivetrain.autoThetaPIDConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(1.5));
    }

    @Override
    public void periodic() {
//        SmartDashboard.putNumber("Limelight Robot Radius", shooterLimeLight.getRadialDistance());
//        spindexer.setPower(1);
//        spindexer.setBallStop(Spindexer.BallStopState.Disabled);
//        shooter.setFlywheelVelocity(SmartDashboard.getNumber("Flywheel Velocity Goal", 5400));
//        SmartDashboard.putNumber("Flywheel Velocity", shooter.getFlywheelVelocity());
//        shooter.setHoodAngle(SmartDashboard.getNumber("Hood Angle", 10.0));

//        drivetrain.setAllModuleRotations(new Rotation2d(0));
//        double forward = 0.75 * -ControlsManager.getAxis(ControlsManager.Control.DriveAxis, 0.2);
//        double strafe = 0.75 * -ControlsManager.getAxis(ControlsManager.Control.StrafeAxis, 0.2);
//        double turn = (3 * -ControlsManager.getAxis(ControlsManager.Control.RotationAxis, 0.2));
//
//        if(ControlsManager.getButton(ControlsManager.Control.ShooterInitiationLine))
//            running = true;

//        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turn, drivetrain.getRobotAngle());
//        ChassisSpeeds speeds;
//        if(running)
//            drivetrain.setAllModuleDriveVelocity(3);
//        else
//            drivetrain.setAllModuleDriveVelocity(0);
        // drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(forward, strafe, turn));
//        drivetrain.driveTranslationRotationRaw(speeds);
//
//        // intake
//
////        System.out.println("prev " + intakeButtonPreviouslyPressed);
////        System.out.println("cur  " + ControlsManager.getButton(ControlsManager.Control.IntakeButton));
//
//        if(ControlsManager.getButton(ControlsManager.Control.IntakeButton) && !intakeButtonPreviouslyPressed) {
//            if(intakePreviouslyDeployed) {
//                intake.setIntakeState(Intake.IntakeState.Retracted);
//
//                intakePreviouslyDeployed = false;
////                LEDs.getInstance().setState(LEDs.State.Yellow);
//            } else {
//                intake.setIntakeState(Intake.IntakeState.Running);
//
//                intakePreviouslyDeployed = true;
////                LEDs.getInstance().enableDefaultState();
//            }
//        }
//        intakeButtonPreviouslyPressed = ControlsManager.getButton(ControlsManager.Control.IntakeButton);
//
////        if (ControlsManager.getButton(ControlsManager.Control.XLockButton)) {
////            shooter.setHoodAngle(0);
////        }
////
////        if (ControlsManager.getButton(ControlsManager.Control.ShooterInitiationLine)) {
////            shooter.setHoodAngle(15);
////        }
////
////        if (ControlsManager.getButton(ControlsManager.Control.ShooterTrenchLine)) {
////            shooter.setHoodAngle(30);
////        }
//
////        spindexer.setPower(ControlsManager.getButton(ControlsManager.Control.HeadingTargetButton) ? 1 : 0);
//        spindexer.setPower(1);
//        spindexer.setBallStop(ControlsManager.getButton(ControlsManager.Control.HeadingTargetButton) ? Spindexer.BallStopState.Disabled : Spindexer.BallStopState.Enabled);
//
////        shooter.setFlywheelVelocity(3000);
////        shooter.setPreRollerPower(1);

//        drivetrain.setAllModuleRotations(new Rotation2d());
//        drivetrain.setAllModuleDriveVelocity(-Units.feetToMeters(4));

//        if (ControlsManager.getButton(ControlsManager.Control.ShooterInitiationLine)) {
//            shooter.setHoodAngle(15);
//        }
//
//        if (ControlsManager.getButton(ControlsManager.Control.ShooterTrenchLine)) {
//            shooter.setHoodAngle(30);
//        }
//
//        spindexer.setPower(ControlsManager.getButton(ControlsManager.Control.HeadingTargetButton) ? 1 : 0);

//        double turn = thetaController.calculate(drivetrain.getPose().getRotation().getRadians(), 0);
//        double x = MathUtil.clamp(xController.calculate(-drivetrain.getPose().getX(), 0), -0.5, 0.5);
//        double y = MathUtil.clamp(yController.calculate(-drivetrain.getPose().getY(), 0), -0.5, 0.5);
//
//        SmartDashboard.putNumber("turn", turn);
//        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(x, y, turn));

//        shooter.setPreset(Constants.ShooterPresets.AutoShoot);
//        spindexer.setPower(1);

//        drivetrain.driveTranslationRotationVelocity(new ChassisSpeeds(Units.feetToMeters(8), 0, 0));

        double turn = thetaController.calculate(drivetrain.getRobotAngle().getRadians(), target.getRadians());

        drivetrain.driveTranslationRotationVelocity(new ChassisSpeeds(0, 0, turn));

        SmartDashboard.putNumber("Angle Target", target.getDegrees());
    }

    @Override
    public void end() {
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds());
        shooter.setFlywheelVelocity(0);
        shooter.setHoodAngle(0);
        shooter.setPreRollerPower(0);
    }
}
