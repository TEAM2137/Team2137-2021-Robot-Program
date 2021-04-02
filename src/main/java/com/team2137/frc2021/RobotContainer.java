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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;


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

    public static SendableChooser<CommandBase> autoSelector;

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

        CommandRunner.registerSubSystem(drivetrain, shooter, shooterLimeLight, ballLimelight, intake, spindexer, ballLimelight, leds);

        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption("Nothing", new InstantCommand(() -> DriverStation.reportWarning("Doing nothing for auto", false)));
        autoSelector.addOption("Galactic Search", new GalacticSearch(drivetrain, intake, spindexer, ballLimelight));
        autoSelector.addOption("Barrel Racing", new BarrelRacing(drivetrain));
        autoSelector.addOption("Slalom Path", new SlalomPath(drivetrain));
        autoSelector.addOption("Bounce Path", new BouncePath(drivetrain));

        SmartDashboard.putData("Auto Selector", autoSelector);
    }
}
