package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.program.ControlsManager.Control;
import com.team2137.frc2021.subsystems.LEDs;
import com.team2137.frc2021.subsystems.LimeLight;
import com.team2137.frc2021.subsystems.Spindexer;
import com.team2137.libs.Util;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import com.team2137.frc2021.subsystems.Intake.IntakeState;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Teleop extends RobotContainer implements OpMode {
    private boolean intakeButtonPreviouslyPressed = false;
    private boolean intakePreviouslyDeployed = false;

    private ProfiledPIDController thetaController;

    private boolean boolHeadingControlEnabled = true;
    private boolean boolDriverTurnedRobot = false;

    @Override
    public void init() {
        drivetrain.selfTargetAllModuleAngles();

        thetaController = new ProfiledPIDController(Constants.Drivetrain.teleopThetaPIDConstants.getP(),
                Constants.Drivetrain.teleopThetaPIDConstants.getI(),
                Constants.Drivetrain.teleopThetaPIDConstants.getD(),
                Constants.Drivetrain.teleopThetaPIDConstraints);

        thetaController.setTolerance(Math.toRadians(4));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        spindexer.setBallStop(Spindexer.BallStopState.Enabled);
        spindexer.setPower(0);
        drivetrain.setBrakeMode(true);

        shooter.setFlywheelVelocity(5000);
        shooter.idleFlyWheel();
        shooter.zeroHoodAngle();

        thetaController.reset(0);

//        SmartDashboard.putNumber("PTheta", Constants.Drivetrain.teleopThetaPIDConstants.getP());
//        SmartDashboard.putNumber("ITheta", Constants.Drivetrain.teleopThetaPIDConstants.getI());
//        SmartDashboard.putNumber("DTheta", Constants.Drivetrain.teleopThetaPIDConstants.getD());

//        shooter.zeroHoodAngle();
        LEDs.getInstance().setDefaultState(LEDs.State.BlinkBlue, true);

//        SmartDashboard.getEntry("thetaController").addListener((table, key, entry, value, flags) -> {
//            thetaController.
//        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
//        shooter.setHoodAngle(26.0);

        thetaController.setGoal(drivetrain.getRobotAngle().getRadians());
    }

    @Override
    public void periodic() {

//        thetaController.setP(SmartDashboard.getNumber("PTheta", Constants.Drivetrain.teleopThetaPIDConstants.getP()));
//        thetaController.setI(SmartDashboard.getNumber("ITheta", Constants.Drivetrain.teleopThetaPIDConstants.getI()));
//        thetaController.setD(SmartDashboard.getNumber("DTheta", Constants.Drivetrain.teleopThetaPIDConstants.getD()));

        double forward;
        double strafe;
        double turn;

        if(!ControlsManager.getButton(Control.SlowButton)) {
            forward = 0.80 * Constants.squareWithSign(-ControlsManager.getAxis(ControlsManager.Control.DriveAxis, 0.2));
            strafe = 0.75 * Constants.squareWithSign(-ControlsManager.getAxis(ControlsManager.Control.StrafeAxis, 0.2));
            turn = 0.7 * Constants.squareWithSign(-ControlsManager.getAxis(ControlsManager.Control.RotationAxis, 0.2));
        } else {
            forward = 0.50 * Constants.squareWithSign(-ControlsManager.getAxis(ControlsManager.Control.DriveAxis, 0.2));
            strafe = 0.50 * Constants.squareWithSign(-ControlsManager.getAxis(ControlsManager.Control.StrafeAxis, 0.2));
            turn = 0.7 * Constants.squareWithSign(-ControlsManager.getAxis(ControlsManager.Control.RotationAxis, 0.2));
        }

        if (SmartDashboard.getBoolean("Baby Mode", false)) {
            forward *= 0.5;
            strafe *= 0.5;
        }

//        if(forward == 0 && strafe == 0 && turn == 0 && ControlsManager.getButton(Control.XLockButton)) {
//            drivetrain.xLock();
//        }

        if (SmartDashboard.getBoolean("Baby Mode", false)) {
            if (ControlsManager.getButton(Control.ShooterStage1)) {
                shooter.setFlywheelVelocity(3500);
                shooter.setHoodAngle(0);
                shooter.revFlywheel();
            } else if (ControlsManager.getButton(Control.ShooterStage2)) {
                shooter.setFlywheelVelocity(3500);
                shooter.setHoodAngle(15);
                shooter.revFlywheel();
            } else if (ControlsManager.getButton(Control.ShooterStage3)) {
                shooter.setFlywheelVelocity(4500);
                shooter.setHoodAngle(0);
                shooter.revFlywheel();
            } else if (ControlsManager.getButton(Control.ShooterStage4)) {
                shooter.setFlywheelVelocity(4500);
                shooter.setHoodAngle(15);
                shooter.revFlywheel();
            } else {
                shooter.idleFlyWheel();
            }
        } else {
            if (ControlsManager.getButton(Control.ShooterStage1)) {
                shooter.setShooterPosisition(5);
            } else if (ControlsManager.getButton(Control.ShooterStage2)) {
                shooter.setShooterPosisition(10);
            } else if (ControlsManager.getButton(Control.ShooterStage3)) {
                shooter.setShooterPosisition(15);
            } else if (ControlsManager.getButton(Control.ShooterStage4)) {
                shooter.setShooterPosisition(20);
            } else if (ControlsManager.getButton(Control.ShooterLimeLight)) {
                shooter.setShooterPosisition(shooterLimeLight.getRadialDistance());
            } else {
                shooter.idleFlyWheel();
            }
        }

//        boolean isPoweringShooter = ControlsManager.getButton(Control.ShooterStage1) || ControlsManager.getButton(Control.ShooterStage2) || ControlsManager.getButton(Control.ShooterStage3) || ControlsManager.getButton(Control.ShooterStage4);

        if(ControlsManager.getButton(Control.HeadingTargetButton) && shooterLimeLight.hasValidTarget()) {
            //Set the flywheel velocity for the shooter and the angle for the hood
//            shooter.setShooterPosisition(shooterLimeLight.getRadialDistance());
//            shooter.setPreRollerPower(1);

            thetaController.setGoal(drivetrain.getRobotAngle().getRadians());

//            double thetaPower = thetaController.calculate(shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX), 0);
//            double thetaPower = -shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX) * 1.8 + 0.5 * Math.signum(Util.deadband(-shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX), 0.5));
            double thetaPower = -shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX) * 1.4;
            if (Math.abs(shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX)) < 0.5) {
                thetaPower += Math.signum(-shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX)) * 0.1;
            }
            SmartDashboard.putNumber("theta", thetaPower);
            drivetrain.driveTranslationRotationRaw(ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, thetaPower, drivetrain.getPose().getRotation()));

//            if(shooterLimeLight.getLimeLightValue(LimeLight.LimeLightValues.TX) < 3 && shooter.isFlywheelAtTarget(100))
//                spindexer.setBallStop(Spindexer.BallStopState.Disabled);

            SmartDashboard.putBoolean("aligning", true);

        } else {
            //Non shooting condition (normal driver mode)
            SmartDashboard.putBoolean("aligning", false);
            double tmp = thetaController.calculate(drivetrain.getRobotAngle().getRadians());

            if(!Constants.withinClip(ControlsManager.getAxis(ControlsManager.Control.RotationAxis), 0, 0.1)) {
                boolDriverTurnedRobot = true;
                boolHeadingControlEnabled = false;
            } else if (boolHeadingControlEnabled) {
                if(!thetaController.atGoal())
                    turn = tmp;
                else
                    turn = 0;
            }

            if (Math.abs(drivetrain.getThetaVelocity()) < 2 && boolDriverTurnedRobot) {
                boolHeadingControlEnabled = true;
                thetaController.reset(drivetrain.getRobotAngle().getRadians());
                thetaController.setGoal(drivetrain.getRobotAngle().getRadians());
                boolDriverTurnedRobot = false;
            }

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turn, drivetrain.getPose().getRotation());
            drivetrain.driveTranslationRotationRaw(speeds);

        }

//        if (ControlsManager.getButton(Control.PreRoller)) {
//            shooter.setPreRollerPower(1);
//        } else {
//            shooter.setPreRollerPower(0);
//        }

//        if (shooter.isFlywheelAtTarget(200) && !shooter.isIdle() && (ControlsManager.getButton(Control.HeadingTargetButton) || ControlsManager.getButton(Control.PreRoller)) && isPoweringShooter) {
        if (shooter.isFlywheelAtTarget(150)) {
            spindexer.setBallStop(Spindexer.BallStopState.Disabled);
            spindexer.setPower(1);
            SmartDashboard.putBoolean("Ball Stopper", false);
        } else {
            spindexer.setBallStop(Spindexer.BallStopState.Enabled);
            spindexer.setPower(0);
            SmartDashboard.putBoolean("Ball Stopper", true);
        }

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

        if(ControlsManager.getButton(Control.IntakeDeploy) && !intakeButtonPreviouslyPressed) {
            if(intakePreviouslyDeployed) {
                intake.setIntakeState(IntakeState.Retracted);
                intakePreviouslyDeployed = false;
//                LEDs.getInstance().setState(LEDs.State.Yellow);
            } else {
                intake.setIntakeState(IntakeState.Deployed);
                intakePreviouslyDeployed = true;
//                LEDs.getInstance().enableDefaultState();
            }
        }

        intakeButtonPreviouslyPressed = ControlsManager.getButton(Control.IntakeButton) || ControlsManager.getButton(Control.IntakeDeploy);
//
//        if(isPoweringShooter) {
//            spindexer.setPower(1);
//        } else {
//            spindexer.setPower(0);
//        }
    }

    @Override
    public void end() {
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, 0));
    }
}
