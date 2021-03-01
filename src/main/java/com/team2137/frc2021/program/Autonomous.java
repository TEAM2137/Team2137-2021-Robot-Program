package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand.HeadingControlThreshold;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.libs.TrajectoryUtility;
import com.team2137.libs.UnitsExtra;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;

public class Autonomous extends RobotContainer implements OpMode {

    private State state = State.Searching;
    NetworkTable llTable = NetworkTableInstance.getDefault().getTable("ballCamera");
    NetworkTableEntry hasTarget = llTable.getEntry("tv");
    NetworkTableEntry ballX = llTable.getEntry("tx");
    NetworkTableEntry ballY = llTable.getEntry("ty");

    PIDController ballSearchTurnController = new PIDController(0.1, 0 ,0);

    Timer intakeTimer = new Timer();

    PIDController xController = new PIDController(Constants.Drivetrain.translationPIDConstants.getP(), Constants.Drivetrain.translationPIDConstants.getI(), Constants.Drivetrain.translationPIDConstants.getD());
    PIDController yController = new PIDController(Constants.Drivetrain.translationPIDConstants.getP(), Constants.Drivetrain.translationPIDConstants.getI(), Constants.Drivetrain.translationPIDConstants.getD());
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.Drivetrain.autoThetaPIDConstants.getP(), Constants.Drivetrain.autoThetaPIDConstants.getD(), Constants.Drivetrain.autoThetaPIDConstants.getD(), Constants.Drivetrain.autoThetaPIDConstraints);

    Pose2d shootingPosition = new Pose2d(new Translation2d(10, 0), Rotation2d.fromDegrees(180));

    @Override
    public void init() {
        new SetIntakeCommand(intake, Intake.IntakeState.Running).schedule();

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(0.05);
    }

    @Override
    public void periodic() {
        switch (state) {
            case Searching:


                if(hasTarget.getDouble(0) == 0) {
                    drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, 0.5));
                } else {
                    double turn = ballSearchTurnController.calculate(-ballX.getDouble(0), 0);
                    double drive = 0.2;
                    drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, drive, turn));
                }

                if(intake.getIntakeCurrentDraw() > 5) {
                    intakeTimer.reset();
                    intakeTimer.start();
                }

                if(intakeTimer.hasElapsed(1)) {
                    state = State.DrivingToGoal;
                }
                break;
            case DrivingToGoal:

//                xController.calculate()
                break;
        }
    }

    @Override
    public void end() {

    }

    private enum State {
        Searching,
        DrivingToGoal
    }
}
