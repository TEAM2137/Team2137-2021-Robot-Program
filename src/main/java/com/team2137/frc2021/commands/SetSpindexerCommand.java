package com.team2137.frc2021.commands;

import com.team2137.frc2021.subsystems.Spindexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSpindexerCommand extends CommandBase {
    private Spindexer spindexer;
    private double power;

    public SetSpindexerCommand(Spindexer spindexer, double power) {
        this.spindexer = spindexer;
        this.power = power;

        addRequirements(spindexer);
    } 

    @Override
    public void initialize() {
        spindexer.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
