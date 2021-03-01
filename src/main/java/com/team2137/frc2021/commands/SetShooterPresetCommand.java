package com.team2137.frc2021.commands;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetShooterPresetCommand extends CommandBase {

    Shooter shooter;
    Constants.ShooterPresets preset;

    public SetShooterPresetCommand(Shooter shooter, Constants.ShooterPresets preset) {
        this.shooter = shooter;
        this.preset = preset;
    }

    @Override
    public void initialize() {
        shooter.setPreset(Constants.ShooterPresets.AutoShoot);
    }


}
