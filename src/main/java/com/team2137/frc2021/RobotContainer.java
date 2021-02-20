package com.team2137.frc2021;

import com.team2137.frc2021.subsystems.LimeLight;
import com.team2137.frc2021.subsystems.Shooter;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;

public class RobotContainer {
    public static SwerveDrivetrain drivetrain;
    public static Shooter shooter;
    public static LimeLight limeLight;

    public static void initialize() {
        drivetrain = new SwerveDrivetrain();
        shooter = new Shooter();
        limeLight = new LimeLight();
    }
}
