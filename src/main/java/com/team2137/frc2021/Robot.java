// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2137.frc2021;

import com.team2137.frc2021.program.*;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    OpMode autonomous = new Autonomous();
    OpMode teleop = new Teleop();
    OpMode disabled = new Disabled();
    OpMode robotMode = new RobotMode();
    OpMode test = new Test();

    OpMode currentOpMode;

    @Override
    public void robotInit() {
        robotMode.init();
    }

    @Override
    public void robotPeriodic() {
        robotMode.periodic();
    }

    @Override
    public void autonomousInit() {
        disabled.end();
        currentOpMode = autonomous;
        autonomous.init();
    }

    @Override
    public void autonomousPeriodic() {
        autonomous.periodic();
    }

    @Override
    public void teleopInit() {
        disabled.end();
        currentOpMode = teleop;
        teleop.init();
    }

    @Override
    public void teleopPeriodic() {
        teleop.periodic();
    }

    @Override
    public void disabledInit() {
        if (currentOpMode != null)
            currentOpMode.end();
        disabled.init();
    }

    @Override
    public void disabledPeriodic() {
        disabled.periodic();
    }

    @Override
    public void testInit() {
        disabled.end();
        currentOpMode = test;
        test.init();
    }

    @Override
    public void testPeriodic() {
        test.periodic();
    }
}
