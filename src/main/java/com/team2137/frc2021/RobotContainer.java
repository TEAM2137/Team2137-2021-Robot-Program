package com.team2137.frc2021;

import com.team2137.frc2021.subsystems.*;

import com.ctre.phoenix.CANifier;

import java.util.List;

public class RobotContainer {

    public static CANifier canifier;

    public static SwerveDrivetrain drivetrain;
    public static Intake intake;
    public static Spindexer spindexer;
    public static Shooter shooter;
    public static LimeLight limeLight;

    public static void initialize() {
        canifier = new CANifier(Constants.canifierID);

        LEDs.initialize(canifier);

        drivetrain = new SwerveDrivetrain();
        shooter = new Shooter();
        limeLight = new LimeLight();
        intake = new Intake();
        spindexer = new Spindexer(canifier);
    }
}
