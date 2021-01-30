package com.team2137.frc2021;

import com.team2137.frc2021.subsystems.SwerveDrivetrain;

public class RobotContainer {
    public static SwerveDrivetrain drivetrain;

    public static void initialize() {
        drivetrain = new SwerveDrivetrain();
    }
}
