package com.team2137.frc2021.program;

import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotMode extends RobotContainer implements OpMode {
    @Override
    public void init() {
        RobotContainer.initialize();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void periodic() {
//        CommandScheduler.getInstance().run();
        drivetrain.periodic();
        intake.periodic();
    }

    @Override
    public void end() {} //do not use, does nothing in RobotMode
}
