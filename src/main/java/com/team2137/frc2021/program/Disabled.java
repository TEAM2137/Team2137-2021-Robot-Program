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
//        CommandScheduler.getInstance().cancelAll();
//        LEDs.getInstance().setDefaultState(LEDs.State.RainbowCycle, true);
        limeLight.disableLED();
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds());
        spindexer.setPower(0);
        shooter.setPreset(Constants.ShooterPresets.Off);
        intake.setIntakeState(Intake.IntakeState.Retracted);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void end() {

        limeLight.enableLED();
    }
}
