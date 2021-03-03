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
import edu.wpi.first.wpilibj.DriverStation;
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

    boolean hasHadTarget = false;
    double prevY = 0;

    PIDController ballSearchTurnController = new PIDController(0.03, 0 ,0);

    Timer intakeStartTimer = new Timer();
    Timer intakeTimer = new Timer();
    Timer switchTimer = new Timer();
    Timer shootTimer = new Timer();

    PIDController xController = new PIDController(Constants.Drivetrain.translationPIDConstants.getP(), Constants.Drivetrain.translationPIDConstants.getI(), Constants.Drivetrain.translationPIDConstants.getD());
    PIDController yController = new PIDController(Constants.Drivetrain.translationPIDConstants.getP(), Constants.Drivetrain.translationPIDConstants.getI(), Constants.Drivetrain.translationPIDConstants.getD());
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.Drivetrain.teleopThetaPIDConstants.getP(), Constants.Drivetrain.teleopThetaPIDConstants.getD(), Constants.Drivetrain.teleopThetaPIDConstants.getD(), Constants.Drivetrain.autoThetaPIDConstraints);

    Pose2d shootingPosition = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180));

    @Override
    public void init() {
        state = State.Searching;

        intakeStartTimer.reset();
        intakeTimer.reset();
        switchTimer.stop();
        switchTimer.reset();

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        thetaController.setTolerance(0.1);

        spindexer.setPower(0);

        hasHadTarget = false;

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

        SmartDashboard.putString("state", state.toString());

        switch (state) {
            case Searching:
                intakeStartTimer.start();

                intake.setIntakeState(Intake.IntakeState.Running);
                spindexer.setBallStop(Spindexer.BallStopState.Enabled);

                if((hasTarget && ballY < 0) || (hasHadTarget)) {
                    SmartDashboard.putString("status", "seen");
                    double turn = 0;
                    if(hasTarget && ballY < 0) {
                        turn = ballSearchTurnController.calculate(-ballX, -5);
                    }
                    double drive = 0.2;
                    drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(drive, 0, -turn));
                    hasHadTarget = true;
                    prevY = ballY;
                } else {
                    SmartDashboard.putString("status", "unknown");
                    drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, 0.5));
                }

                if(intake.getIntakeCurrentDraw() > 5 && intakeStartTimer.hasElapsed(0.5)) {
                    intakeTimer.reset();
                    intakeTimer.start();
                    switchTimer.reset();
                    switchTimer.start();
                    DriverStation.reportError("HIT", false);
                }

                SmartDashboard.putNumber("timer", intakeTimer.get());
                if(intakeTimer.hasElapsed(0.01) && intakeStartTimer.hasElapsed(0.5)) {

                }

                if(switchTimer.hasElapsed(.5)) {
                    state = State.DrivingToGoal;
                    switchTimer.stop();
                    switchTimer.reset();
                }
                break;
            case DrivingToGoal:
                intake.setIntakeState(Intake.IntakeState.Retracted);

                double xPower = xController.calculate(drivetrain.getPose().getX(), 0);
                double yPower = yController.calculate(drivetrain.getPose().getY(), 0);
                double thetaPower = thetaController.calculate(drivetrain.getPose().getRotation().getRadians(), 180);

                drivetrain.driveTranslationRotationRaw(new ChassisSpeeds());

                spindexer.setPower(1);
                shooter.setPreset(Constants.ShooterPresets.AutoShoot);

//                shooter.setPreset(Constants.ShooterPresets.AutoShoot);

//                drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(xPower, yPower, thetaPower));

                if(xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) {
                    spindexer.setBallStop(Spindexer.BallStopState.Disabled);
                    shootTimer.start();
                }

                if(shootTimer.hasElapsed(.5)) {
//                    spindexer.setBallStop(Spindexer.BallStopState.Enabled);
//                    shooter.setFlywheelVelocity(0);
                    shooter.setHoodAngle(0);
//                    shooter.setPreRollerPower(0);
                    state = State.Searching;
                    shootTimer.stop();
                    shootTimer.reset();
                    intakeStartTimer.stop();
                    intakeStartTimer.reset();
                }
                break;
        }


    }

    @Override
    public void end() {
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds());
        shooter.setHoodAngle(0);
        shooter.setFlywheelVelocity(0);
        shooter.setPreRollerPower(0);
    }

    private enum State {
        Searching,
        DrivingToGoal
    }
}
