package com.team2137.frc2021.commands;

import com.team2137.frc2021.subsystems.Spindexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpindexerCommand extends CommandBase {
    private Spindexer spindexer;
    private boolean enabled;

    public SpindexerCommand(Spindexer spindexer, boolean enable) {
        this.spindexer = spindexer;
        this.enabled = enable;

        addRequirements(spindexer);
    } 

    @Override
    public void initialize() {
        if (enabled) {
            spindexer.disabledBallStopper();
        } else {
            spindexer.enableBallStopper();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
