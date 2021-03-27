package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.LEDs;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Disabled extends RobotContainer implements OpMode {
    @Override
    public void init() {
//        LEDs.getInstance().setDefaultState(LEDs.State.RainbowCycle, true);
//        shooterLimeLight.disableLED();
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds());
        spindexer.setPower(0);
        shooter.setPreset(Constants.ShooterPresets.Off);
        intake.setIntakeState(Intake.IntakeState.Retracted);

        LEDs.getInstance().setState(LEDs.State.TeamColorCycle);
    }

    @Override
    public void periodic() {
//        if(ControlsManager.getButton(ControlsManager.Control.HeadingTargetButton)) {
//            LEDs.getInstance().setState(LEDs.State.Blue);
//        }
//        if(ControlsManager.getButton(ControlsManager.Control.IntakeButton)) {
//            LEDs.getInstance().setState(LEDs.State.TeamColorCycle);
//        }
    }

    @Override
    public void end() {

        shooterLimeLight.enableLED();
    }
}
