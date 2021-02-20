package com.team2137.frc2021;

import com.team2137.frc2021.subsystems.*;

import com.ctre.phoenix.CANifier;

public class RobotContainer {

    public static CANifier canifier;

    public static SwerveDrivetrain drivetrain;
    public static Intake intake;
    public static Spindexer spindexer;



    public static void initialize() {
        canifier = new CANifier(Constants.canifierID);

        drivetrain = new SwerveDrivetrain();
        intake = new Intake();
        spindexer = new Spindexer(canifier);
    }
}
