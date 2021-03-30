package com.team2137.frc2021;

import com.team2137.frc2021.autonomous.BarrelRacing;
import com.team2137.frc2021.autonomous.BouncePath;
import com.team2137.frc2021.autonomous.GalacticSearch;
import com.team2137.frc2021.autonomous.SlalomPath;
import com.team2137.frc2021.program.Autonomous;
import com.team2137.frc2021.subsystems.*;

import com.ctre.phoenix.CANifier;
import com.team2137.frc2021.util.CommandRunner;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.team2137.frc2021.program.Autonomous.Challenge;

import java.sql.Driver;
import java.util.function.Consumer;

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

    public static RobotContainer robotInstance;
    public static SendableChooser<Autonomous.Challenge> autoSelector;

    public RobotContainer() {
        robotInstance = this;

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
        CommandRunner.registerSubSystem(ballLimelight);
        CommandRunner.registerSubSystem(leds);

        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption("Nothing", Challenge.Unknown);
        autoSelector.addOption("Galactic Search", Challenge.GalacticSearch);
        autoSelector.addOption("Barrel Racing", Challenge.BarrelRacing);
        autoSelector.addOption("Slalom Path", Challenge.SlalomPath);
        autoSelector.addOption("Bounce Path", Challenge.BouncePath);

        SmartDashboard.putData("Auto Selector", autoSelector);
        NetworkTableInstance.getDefault().addEntryListener("Auto Selector", entryNotification -> {
            switch (entryNotification.name) {
                case "Nothing":
                    break;
                case "Galactic Search":
                    new GalacticSearch(robotInstance);
                    DriverStation.reportWarning("Running Galactic", false);
                    break;
                case "Barrel Racing":
                    new BarrelRacing(robotInstance);
                    DriverStation.reportWarning("Running Barrel", false);
                    break;
                case "Slalom Path":
                    new SlalomPath(robotInstance);
                    DriverStation.reportWarning("Running Slalom", false);
                    break;
                case "Bounce Path":
                    new BouncePath(robotInstance);
                    DriverStation.reportWarning("Running Bounce", false);
                    break;
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
}
