package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.program.ControlsManager.Control;
import com.team2137.frc2021.subsystems.LEDs;
import com.team2137.frc2021.subsystems.LimeLight;
import com.team2137.frc2021.subsystems.Spindexer;
import com.team2137.frc2021.util.PID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import com.team2137.frc2021.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.util.Timer;
import java.util.TreeMap;

public class Teleop extends RobotContainer implements OpMode {
    private boolean intakeButtonPreviouslyPressed = false;
    private boolean intakePreviouslyDeployed = false;

    private ProfiledPIDController thetaController;

    private boolean boolDriverControlledTurning = true;
    private Rotation2d dblDriveTrainTurnGoal = new Rotation2d();

    @Override
    public void init() {
        drivetrain.selfTargetAllModuleAngles();

        thetaController = new ProfiledPIDController(Constants.Drivetrain.teleopThetaPIDConstants.getP(),
                Constants.Drivetrain.teleopThetaPIDConstants.getI(),
                Constants.Drivetrain.teleopThetaPIDConstants.getD(),
                Constants.Drivetrain.teleopThetaPIDConstraints);

        thetaController.setTolerance(Math.toRadians(2.5));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        spindexer.setBallStop(Spindexer.BallStopState.Enabled);
        spindexer.setPower(1);
        drivetrain.setBrakeMode(true);

        shooter.setFlywheelVelocity(3000);
        shooter.idleFlyWheel();
//        shooter.zeroHoodAngle();
//        LEDs.getInstance().setDefaultState(LEDs.State.Blue, true);
//        shooter.setHoodAngle(26.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight Robot Radius", shooterLimeLight.getRadialDistance());
//        drivetrain.setAllModuleRotations(new Rotation2d(0));
//        double forward = 0.75 * Constants.squareWithSign(-ControlsManager.getAxis(ControlsManager.Control.DriveAxis, 0.2));
        double forward = 0.75 * -ControlsManager.getAxis(ControlsManager.Control.DriveAxis, 0.2);
        double strafe = 0.75 * Constants.squareWithSign(-ControlsManager.getAxis(ControlsManager.Control.StrafeAxis, 0.2));
        double turn = 3 * Constants.squareWithSign(-ControlsManager.getAxis(ControlsManager.Control.RotationAxis, 0.2));

        // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
        if(forward == 0 && strafe == 0 && turn == 0 && ControlsManager.getButton(Control.XLockButton)) {
            drivetrain.xLock();
        }

        if (ControlsManager.getButton(Control.ShooterStage1) && shooter.isIdle()) {
            shooter.setShooterPosisition(5);
            shooter.revFlywheel();
        } else if (ControlsManager.getButton(Control.ShooterStage2) && shooter.isIdle()) {
            shooter.setShooterPosisition(10);
            shooter.revFlywheel();
        } else if (ControlsManager.getButton(Control.ShooterStage3) && shooter.isIdle()) {
            shooter.setShooterPosisition(15);
            shooter.revFlywheel();
        } else if (ControlsManager.getButton(Control.ShooterStage4) && shooter.isIdle()) {
            shooter.setShooterPosisition(20);
            shooter.revFlywheel();
        } else if(ControlsManager.getButton(Control.HeadingTargetButton) && shooterLimeLight.hasValidTarget()) {
            //Set the flywheel velocity for the shooter and the angle for the hood
            shooter.setShooterPosisition(shooterLimeLight.getRadialDistance());
            shooter.setPreRollerPower(1);

            double thetaPower = thetaController.calculate(shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX));
            drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(forward, strafe, thetaPower));

//            if(shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX) < 3 && shooter.isFlywheelAtTarget(100))
//                spindexer.setBallStop(Spindexer.BallStopState.Disabled);

        } else {
            //Non shooting condition (normal driver mode)

            if(!shooter.isIdle())
                shooter.idleFlyWheel();

            SmartDashboard.putNumber("DriveTurn Angle", dblDriveTrainTurnGoal.getDegrees());

            if(Constants.withinClip(turn, 0, 0.05) && !boolDriverControlledTurning) {
                turn = thetaController.calculate(drivetrain.getRobotAngle().getRadians()) * 0.25;
            } else if(Constants.withinClip(turn, 0, 0.05) && boolDriverControlledTurning && drivetrain.getThetaVelocity() < 5) {
                boolDriverControlledTurning = false;
                SmartDashboard.putBoolean("TURNING", false);
                thetaController.setGoal(dblDriveTrainTurnGoal.getRadians());
                dblDriveTrainTurnGoal = drivetrain.getRobotAngle();
            } else if(!Constants.withinClip(turn, 0, 0.05) && !boolDriverControlledTurning) {
                boolDriverControlledTurning = true;
                SmartDashboard.putBoolean("TURNING", true);
            }

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turn, drivetrain.getRobotAngle());
            drivetrain.driveTranslationRotationRaw(speeds);
        }

        if (shooter.isFlywheelAtTarget(300)) {
            spindexer.setBallStop(Spindexer.BallStopState.Disabled);
            SmartDashboard.putBoolean("Ball Stopper", false);
        } else {
            spindexer.setBallStop(Spindexer.BallStopState.Enabled);
            SmartDashboard.putBoolean("Ball Stopper", true);
        }

//        drivetrain.setAllModuleRotations(new Rotation2d(Math.atan2(forward, strafe)));
//        drivetrain.setAllModuleRotations(Rotation2d.fromDegrees(0));

        // Intake

        if(ControlsManager.getButton(Control.IntakeButton) && !intakeButtonPreviouslyPressed) {
            if(intakePreviouslyDeployed) {
                intake.setIntakeState(IntakeState.Retracted);
                intakePreviouslyDeployed = false;
//                LEDs.getInstance().setState(LEDs.State.Yellow);
            } else {
                intake.setIntakeState(IntakeState.Running);
                intakePreviouslyDeployed = true;
//                LEDs.getInstance().enableDefaultState();
            }
        }
        intakeButtonPreviouslyPressed = ControlsManager.getButton(Control.IntakeButton);
        spindexer.setPower(1);
    }

    @Override
    public void end() {
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, 0));
    }
}
