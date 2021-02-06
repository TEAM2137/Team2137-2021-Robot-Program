package com.team2137.frc2021.commands;

import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeCommand extends CommandBase {
    private IntakeState state;
    private Intake intake;
    public SetIntakeCommand(Intake intake, IntakeState state) {
        this.intake = intake;
        this.state = state;
    }

    @Override
    public void initialize() {
        intake.setIntakeState(state);
    }   

    @Override
    public boolean isFinished() {
        return true;
    }
}