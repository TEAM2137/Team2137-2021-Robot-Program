package com.team2137.frc2021.program;

import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.util.CommandRunner;

public class RobotMode extends RobotContainer implements OpMode {
    @Override
    public void init() {
        RobotContainer.initialize();
    }

    @Override
    public void periodic() {
//        drivetrain.periodic();
//        shooter.periodic();
//        intake.periodic();
//        spindexer.periodic();
//        shooterLimeLight.periodic();
    }

    @Override
    public void end() {
        CommandRunner.purgeSubSystems();
    } //do not use, does nothing in RobotMode
}
