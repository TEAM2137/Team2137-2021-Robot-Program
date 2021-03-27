package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.Spindexer;
import edu.wpi.first.networktables.NetworkTable;
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
import edu.wpi.first.wpiutil.math.MathUtil;

public class Autonomous extends RobotContainer implements OpMode {

    @Override
    public void init() {
    }

    @Override
    public void periodic() {

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
                        turn = ballSearchTurnController.calculate(-ballX, -6);
                    }
                    double drive = 0.2;
                    drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(drive, 0, -turn));
                    hasHadTarget = true;
                    prevY = ballY;
                } else {
                    SmartDashboard.putString("status", "unknown");
                    drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, 0.5));
                }

                if(intake.getIntakeCurrentDraw() > 10 && intakeStartTimer.hasElapsed(1)) {
                    intakeTimer.reset();
                    intakeTimer.start();
                    switchTimer.reset();
                    switchTimer.start();
                    DriverStation.reportError("HIT", false);
                }

                if(switchTimer.hasElapsed(.5)) {
                    state = State.DrivingToGoal;
                    switchTimer.stop();
                    switchTimer.reset();
                    shooter.setPreset(Constants.ShooterPresets.AutoShoot);
                }
                break;
            case DrivingToGoal:
                intake.setIntakeState(Intake.IntakeState.Retracted);

                double xPower = MathUtil.clamp(xController.calculate(-drivetrain.getPose().getX(), 0), -0.45, 0.45);
                double yPower = MathUtil.clamp(yController.calculate(-drivetrain.getPose().getY(), 0), -0.45, 0.45);
                double thetaPower;

//                if(limeLight.hasTarget()) {
//                    thetaPower = Math.toDegrees(limeLight.getTx()) / 25.0;
//                } else {
                    thetaPower = thetaController.calculate(drivetrain.getPose().getRotation().getRadians(), 0);
//                }

                drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(xPower, yPower, thetaPower));

                spindexer.setPower(0);

                shooter.setPreRollerPower(1);

                if(xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) {
                    DriverStation.reportError("at target", false);
                    shootTimer.start();
                    spindexer.setPower(1);
                }

                if(shootTimer.hasElapsed(6)) {
//                    spindexer.setBallStop(Spindexer.BallStopState.Enabled);
                    spindexer.setPower(0);
                    shooter.setPreset(Constants.ShooterPresets.Off);
                    state = State.Searching;
                    shootTimer.stop();
                    shootTimer.reset();
                    intakeStartTimer.stop();
                    intakeStartTimer.reset();
                    hasHadTarget = false;
                }
                break;
        }


    }

    @Override
    public void end() {

    }
}
