// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2137.frc2021;

import com.team2137.frc2021.program.*;
import edu.wpi.first.wpilibj.TimedRobot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Robot extends TimedRobot {
    OpMode autonomous = new Autonomous();
    OpMode teleop = new Teleop();
    OpMode disabled = new Disabled();
    OpMode robotMode = new RobotMode();
    OpMode test = new Test();

    private static List<Runnable> onEnable = new ArrayList<>();
    private static List<Runnable> onDisabled = new ArrayList<>();

    OpMode currentOpMode;

    public Robot() {
        super(0.025);
    }

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
//        runOnEnabledFunctions();
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
//        runOnEnabledFunctions();
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
//        runOnDisabledFunctions();
//        if (currentOpMode != null)
//            currentOpMode.end();
        disabled.init();
    }

    @Override
    public void disabledPeriodic() {
        disabled.periodic();
    }

    @Override
    public void testInit() {
//        runOnEnabledFunctions();
        disabled.end();
        currentOpMode = test;
        test.init();
    }

    @Override
    public void testPeriodic() {
        test.periodic();
    }

    public static void addOnEnabled(Runnable run) {
        onEnable.add(run);
    }
    public static void removeOnEnabled(Runnable run) {
        onEnable.remove(run);
    }

    private void runOnEnabledFunctions() {
        for (Runnable a : onEnable) {
            a.run();
        }
    }

    public static void addOnDisabled(Runnable run) {
        onDisabled.add(run);
    }
    public static void removeOnDisabled(Runnable run) {
        onDisabled.remove(run);
    }

    private void runOnDisabledFunctions() {
        for (Runnable a : onDisabled) {
            a.run();
        }
    }
}
