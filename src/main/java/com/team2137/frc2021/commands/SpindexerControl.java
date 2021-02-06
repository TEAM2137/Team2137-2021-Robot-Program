package com.team2137.frc2021.commands;

import com.team2137.frc2021.subsystems.Spindexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpindexerControl extends CommandBase {
    Spindexer spindexer;
    Timer mainTimer;
    Timer fifthBallTimer;
    States state;

    public SpindexerControl(Spindexer spindexer) {
        this.spindexer = spindexer;
        this.mainTimer = new Timer();
        this.fifthBallTimer = new Timer();

        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        mainTimer.reset();
        mainTimer.stop();

        fifthBallTimer.reset();
        state = States.Waiting;
    }

    @Override
    public void execute() {
        switch(state) {
            case Waiting:
                fifthBallTimer.stop();
                fifthBallTimer.reset();
                if(spindexer.getHopperFirstPhotoeye() || spindexer.getHopperSecondPhotoeye()) {
                    mainTimer.reset();
                    mainTimer.start();
                    spindexer.setPower(1);
                    state = States.Indexing;

                    fifthBallTimer.stop();
                    fifthBallTimer.reset();
                }
                break;
            case Indexing:
                if(spindexer.getHopperFirstPhotoeye() || spindexer.getHopperSecondPhotoeye()) {
                    mainTimer.reset();
                    mainTimer.start();
                }
                
                if(spindexer.getFifthBallPhotoeye()) {
                    fifthBallTimer.start();
                } else {
                    fifthBallTimer.stop();
                    fifthBallTimer.reset();
                }

                if(fifthBallTimer.hasElapsed(1)) {
                    spindexer.setPower(0);
                    state = States.Waiting;
                    // TODO: add retract intake command
                }

                if(mainTimer.hasElapsed(2)) {
                    spindexer.setPower(0);
                    state = States.Waiting;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.setPower(0);
    }

    private enum States {
        Waiting,
        Indexing,
    }
}