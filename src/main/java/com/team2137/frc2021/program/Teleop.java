package com.team2137.frc2021.program;

import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.program.ControlsManager.Control;
import com.team2137.frc2021.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Teleop extends RobotContainer implements OpMode {
    private boolean intakeButtonPreviouslyPressed = false;
    private boolean intakePreviouslyDeployed = false;

    @Override
    public void init() {
        drivetrain.selfTargetAllModuleAngles();
    }

    @Override
    public void periodic() {
        double forward = 0.75 * -ControlsManager.getAxis(Control.DriveAxis, 0.2);
        double strafe = 0.75 * -ControlsManager.getAxis(Control.StrafeAxis, 0.2);
        double turn = 3 * -ControlsManager.getAxis(Control.RotationAxis, 0.2);
        SmartDashboard.putNumber("forward", forward);
        SmartDashboard.putNumber("strafe", strafe);
        SmartDashboard.putNumber("turn", turn);
        // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turn, drivetrain.getRobotAngle());
        // drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(forward, strafe, turn));
        drivetrain.driveTranslationRotationRaw(speeds);
//        drivetrain.setAllModuleRotations(new Rotation2d(Math.atan2(forward, strafe)));
//        drivetrain.setAllModuleRotations(Rotation2d.fromDegrees(0));


        if(ControlsManager.getButton(Control.IntakeButton) && !intakeButtonPreviouslyPressed) {
            if(intakePreviouslyDeployed) {
                new SetIntakeCommand(intake, IntakeState.Retracted).schedule();
                intakePreviouslyDeployed = false;
            } else {
                new SetIntakeCommand(intake, IntakeState.Running).schedule();
                intakePreviouslyDeployed = true;
            }
        }
        intakeButtonPreviouslyPressed = ControlsManager.getButton(Control.IntakeButton);
    }

    @Override
    public void end() {
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, 0));
    }
}
