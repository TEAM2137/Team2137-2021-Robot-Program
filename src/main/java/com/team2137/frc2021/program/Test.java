package com.team2137.frc2021.program;

import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.subsystems.Intake;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Test extends RobotContainer implements OpMode {
    private boolean intakeButtonPreviouslyPressed = false;
    private boolean intakePreviouslyDeployed = false;

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        double forward = 0.75 * -ControlsManager.getAxis(ControlsManager.Control.DriveAxis, 0.2);
        double strafe = 0.75 * -ControlsManager.getAxis(ControlsManager.Control.StrafeAxis, 0.2);
        double turn = (3 * -ControlsManager.getAxis(ControlsManager.Control.RotationAxis, 0.2));

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turn, drivetrain.getRobotAngle());
        // drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(forward, strafe, turn));
        drivetrain.driveTranslationRotationRaw(speeds);

        // intake

        if(ControlsManager.getButton(ControlsManager.Control.IntakeButton) && !intakeButtonPreviouslyPressed) {
            if(intakePreviouslyDeployed) {
                new SetIntakeCommand(intake, Intake.IntakeState.Retracted).schedule();
                intakePreviouslyDeployed = false;
//                LEDs.getInstance().setState(LEDs.State.Yellow);
            } else {
                new SetIntakeCommand(intake, Intake.IntakeState.Running).schedule();
                intakePreviouslyDeployed = true;
//                LEDs.getInstance().enableDefaultState();
            }
        }
        intakeButtonPreviouslyPressed = ControlsManager.getButton(ControlsManager.Control.IntakeButton);
    }

    @Override
    public void end() {
    }
}
