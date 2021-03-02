package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand;
import com.team2137.frc2021.commands.TrajectoryFollowCommand.HeadingControlThreshold;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.LimeLight;
import com.team2137.frc2021.subsystems.Spindexer;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;

public class Autonomous extends RobotContainer implements OpMode {

    private State state = State.Searching;
    NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight-two");
//    NetworkTableEntry hasTarget = llTable.getEntry("tv");
//    NetworkTableEntry ballX = llTable.getEntry("tx");
//    NetworkTableEntry ballY = llTable.getEntry("ty");

    boolean hasTarget = false;
    double ballX = 0;
    double ballY = 0;

    PIDController ballSearchTurnController = new PIDController(0.1, 0 ,0);

    Timer intakeTimer = new Timer();
    Timer switchTimer = new Timer();

    PIDController xController = new PIDController(Constants.Drivetrain.translationPIDConstants.getP(), Constants.Drivetrain.translationPIDConstants.getI(), Constants.Drivetrain.translationPIDConstants.getD());
    PIDController yController = new PIDController(Constants.Drivetrain.translationPIDConstants.getP(), Constants.Drivetrain.translationPIDConstants.getI(), Constants.Drivetrain.translationPIDConstants.getD());
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.Drivetrain.autoThetaPIDConstants.getP(), Constants.Drivetrain.autoThetaPIDConstants.getD(), Constants.Drivetrain.autoThetaPIDConstants.getD(), Constants.Drivetrain.autoThetaPIDConstraints);

    Pose2d shootingPosition = new Pose2d(new Translation2d(10, 0), Rotation2d.fromDegrees(180));

    @Override
    public void init() {
        state = State.Searching;

        intakeTimer.reset();
        switchTimer.stop();
        switchTimer.reset();

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(0.05);

//        llTable.getEntry("tv").addListener((table) -> {
//            hasTarget = table.getEntry().getDouble(0) >= 1.0;
//        }, 0);
//        llTable.getEntry("tx").addListener((table) -> {
//            ballX = table.getEntry().getDouble(0);
//        }, 0);
//        llTable.getEntry("ty").addListener((table) -> {
//            ballY = table.getEntry().getDouble(0);
//        }, 0);
    }

    @Override
    public void periodic() {
        hasTarget = llTable.getEntry("tv").getDouble(0) > 0;
        ballX = llTable.getEntry("tx").getDouble(0);
        ballY = llTable.getEntry("ty").getDouble(0);

        switch (state) {
            case Searching:
                intake.setIntakeState(Intake.IntakeState.Running);
                spindexer.setBallStop(Spindexer.BallStopState.Enabled);

                if(hasTarget) {
                    SmartDashboard.putString("status", "seen");
                    double turn = ballSearchTurnController.calculate(-ballX + 10, 0);
                    double drive = 0.1;
//                    drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, drive, turn));
                } else {
                    SmartDashboard.putString("status", "unknown");
//                    drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, 0.01));
                }

                if(intake.getIntakeCurrentDraw() < 5) {
                    intakeTimer.reset();
                    intakeTimer.start();
                }

                SmartDashboard.putNumber("timer", intakeTimer.get());
                if(intakeTimer.hasElapsed(0.25)) {
                    switchTimer.reset();
                    switchTimer.start();
                }

                if(switchTimer.hasElapsed(1.5)) {
                    state = State.DrivingToGoal;
                    switchTimer.stop();
                    switchTimer.reset();
                }
                break;
            case DrivingToGoal:
                intake.setIntakeState(Intake.IntakeState.Retracted);

                break;
        }
    }

    @Override
    public void end() {
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds());
    }

    private enum State {
        Searching,
        DrivingToGoal
    }
}
