package com.team2137.frc2021.program;

import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.Spindexer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Test extends RobotContainer implements OpMode {
    private boolean intakeButtonPreviouslyPressed = false;
    private boolean intakePreviouslyDeployed = false;

    @Override
    public void init() {
//        shooter.zeroHoodAngle();

        Compressor c = new Compressor();
//        c.stop();
    }

    @Override
    public void periodic() {
        double forward = 0.75 * -ControlsManager.getAxis(ControlsManager.Control.DriveAxis, 0.2);
        double strafe = 0.75 * -ControlsManager.getAxis(ControlsManager.Control.StrafeAxis, 0.2);
        double turn = (3 * -ControlsManager.getAxis(ControlsManager.Control.RotationAxis, 0.2));

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turn, drivetrain.getRobotAngle());
        // drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(forward, strafe, turn));
        drivetrain.driveTranslationRotationRaw(speeds);
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
        intakeButtonPreviouslyPressed = ControlsManager.getButton(ControlsManager.Control.IntakeButton);
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

    }

    @Override
    public void end() {
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds());
        shooter.setFlywheelVelocity(0);
    }
}
