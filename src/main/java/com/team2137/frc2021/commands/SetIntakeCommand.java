package com.team2137.frc2021.commands;

import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeCommand extends CommandBase {
    private IntakeState state;
    private Intake intake;
    public SetIntakeCommand() {
        intake = new Intake();
        state = intake.getIntakeState();

    }

    @Override
    public void initialize() {
        if (state == IntakeState.Retracted) {
            state = IntakeState.Running;
            intake.setIntakeState(IntakeState.Running);
        }
        else {
            state = IntakeState.Retracted;
            intake.setIntakeState(IntakeState.Retracted);
        }
    }

}