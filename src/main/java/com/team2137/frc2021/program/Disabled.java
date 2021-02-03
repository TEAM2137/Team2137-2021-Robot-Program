package com.team2137.frc2021.program;

import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Disabled extends RobotContainer implements OpMode {
    @Override
    public void init() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void periodic() {

    }

    @Override
    public void end() {

    }
}
