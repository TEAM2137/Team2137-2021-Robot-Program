package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.LEDs;
import com.team2137.frc2021.util.CommandRunner;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Disabled extends RobotContainer implements OpMode {
    Timer disabledTimer = new Timer();

    @Override
    public void init() {
//        LEDs.getInstance().setDefaultState(LEDs.State.RainbowCycle, true);
        shooterLimeLight.disableLED();
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds());
        spindexer.setPower(0);
        shooter.setPreset(Constants.ShooterPresets.Off);
        intake.setIntakeState(Intake.IntakeState.Retracted);
        shooter.setPreset(Constants.ShooterPresets.Off);

         drivetrain.driveTranslationRotationRaw(new ChassisSpeeds());
         spindexer.setPower(0);
         shooter.setPreset(Constants.ShooterPresets.Off);
         intake.setIntakeState(Intake.IntakeState.Retracted);
         shooter.setPreset(Constants.ShooterPresets.Off);

         LEDs.getInstance().setDefaultState(LEDs.State.RainbowCycle, true);

        CommandRunner.purgeCommands();

        disabledTimer.reset();
        disabledTimer.start();
    }

    @Override
    public void periodic() {
        if (disabledTimer.hasElapsed(5)) {
           drivetrain.setBrakeMode(false);
        }
    }

    @Override
    public void end() {
        shooterLimeLight.enableLED();
        drivetrain.setBrakeMode(true);
    }
}
