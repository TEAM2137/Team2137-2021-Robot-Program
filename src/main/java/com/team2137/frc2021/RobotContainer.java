package com.team2137.frc2021;

import com.team2137.frc2021.subsystems.*;

public class RobotContainer {
    public static SwerveDrivetrain drivetrain;
    public static Intake intake;

    public static void initialize() {
        drivetrain = new SwerveDrivetrain();
        intake = new Intake();
    }
}
