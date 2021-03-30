package com.team2137.frc2021;

import com.team2137.frc2021.subsystems.*;

import com.ctre.phoenix.CANifier;
import com.team2137.frc2021.util.CommandRunner;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    public static CANifier canifier;

    public static SwerveDrivetrain drivetrain;
    public static Intake intake;
    public static Spindexer spindexer;
    public static Shooter shooter;
    public static ShooterLimeLight shooterLimeLight;
    public static CommandRunner commandRunner;
    public static BallLimeLight ballLimelight;
    public static LEDs leds;

    public static SendableChooser<Command> autoSelector;

    public static void initialize() {
        canifier = new CANifier(Constants.canifierID);

        leds = new LEDs(canifier);
        commandRunner = new CommandRunner();

        drivetrain = new SwerveDrivetrain();
        shooter = new Shooter();
        shooterLimeLight = new ShooterLimeLight();
        ballLimelight = new BallLimeLight();
        intake = new Intake();
        spindexer = new Spindexer();

        CommandRunner.registerSubSystem(drivetrain);
        CommandRunner.registerSubSystem(shooter);
        CommandRunner.registerSubSystem(shooterLimeLight);
        CommandRunner.registerSubSystem(ballLimelight);
        CommandRunner.registerSubSystem(intake);
        CommandRunner.registerSubSystem(spindexer);
        CommandRunner.registerSubSystem(leds);

//        autoSelector = new SendableChooser<>();
//        autoSelector.addOption("Galactic Search", new GalacticSearch(drivetrain, intake, spindexer, ballLimelight));
    }
}
