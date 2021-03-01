package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.commands.SetSpindexerCommand;
import com.team2137.frc2021.program.ControlsManager.Control;
import com.team2137.frc2021.subsystems.LEDs;
import com.team2137.frc2021.util.PID;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import com.team2137.frc2021.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Teleop extends RobotContainer implements OpMode {
    private boolean intakeButtonPreviouslyPressed = false;
    private boolean intakePreviouslyDeployed = false;

    private ProfiledPIDController headingController;

    @Override
    public void init() {
        drivetrain.selfTargetAllModuleAngles();

        PID headingConstants = Constants.Drivetrain.teleopThetaPIDConstants;
        headingController = new ProfiledPIDController(headingConstants.getP(), headingConstants.getI(), headingConstants.getD(), Constants.Drivetrain.teleopThetaPIDConstraints);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

//        LEDs.getInstance().setDefaultState(LEDs.State.Blue, true);
    }

    @Override
    public void periodic() {

        // Drivetrain
        double forward = 0.75 * -ControlsManager.getAxis(Control.DriveAxis, 0.2);
        double strafe = 0.75 * -ControlsManager.getAxis(Control.StrafeAxis, 0.2);
        double turn = (3 * -ControlsManager.getAxis(Control.RotationAxis, 0.2));

//        if (limeLight.hasTarget() && ControlsManager.getAxis(Control.LimeLightButton, 0) > 0.5) {
//            turn += limeLight.getRotationVector();
//        } else if (ControlsManager.getAxis(Control.LimeLightButton, 0) > 0.5) {
//            turn += .5;
//        }

        SmartDashboard.putNumber("forward", forward);
        SmartDashboard.putNumber("strafe", strafe);
        SmartDashboard.putNumber("turn", turn);
        // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
        if(forward == 0 && strafe == 0 && turn == 0 && ControlsManager.getButton(Control.XLockButton)) {
            drivetrain.xLock();
        } else if(ControlsManager.getButton(Control.HeadingTargetButton)) {

            //Set the flywheel velocity for the shooter and the angle for the hood
            shooter.setFlywheelVelocity(ShooterMap.getFlywheelSpeed(limeLight.getRobotRadius()));
            shooter.setHoodAngle(ShooterMap.getHoodAngle(limeLight.getRobotRadius()));

            Rotation2d angle;
            //If the LimeLight has a target use that target if not use the estimated position using pose
            if (limeLight.hasTarget())
                angle = new Rotation2d(-(limeLight.translateCameraXToCenter(drivetrain.getRobotAngle().getDegrees()) - Constants.Shooter.LimeLightTargetFieldPosition.x), -(limeLight.translateCameraYToCenter(drivetrain.getRobotAngle().getDegrees()) - Constants.Shooter.LimeLightTargetFieldPosition.y));
            else
                angle = new Rotation2d(-(drivetrain.getPose().getX() - Constants.Shooter.LimeLightTargetFieldPosition.x), -(drivetrain.getPose().getY() - Constants.Shooter.LimeLightTargetFieldPosition.y));

            //TODO add an insurance on the pose and if needed add control for rotating until seeing target

            //Power for the robot to turn given the Robot Angle in Radians
            double thetaPower = headingController.calculate(drivetrain.getRobotAngle().getRadians(), angle.getRadians());

            if (thetaPower < 0.05 && shooter.isFlywheelAtTarget(50))
                CommandScheduler.getInstance().schedule(new SetSpindexerCommand(spindexer, 0));

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe,
                    Constants.applyDeadBand(thetaPower, 0.05),
                    drivetrain.getRobotAngle());

            drivetrain.driveTranslationRotationRaw(speeds);

        } else {
            if(!spindexer.isBallStopEnabled()) {
                CommandScheduler.getInstance().schedule(new SetSpindexerCommand(spindexer, 1));
            }
            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, turn, drivetrain.getRobotAngle());
            // drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(forward, strafe, turn));
            drivetrain.driveTranslationRotationRaw(speeds);
        }
//        drivetrain.setAllModuleRotations(new Rotation2d(Math.atan2(forward, strafe)));
//        drivetrain.setAllModuleRotations(Rotation2d.fromDegrees(0));


        // Intake

        if(ControlsManager.getButton(Control.IntakeButton) && !intakeButtonPreviouslyPressed) {
            if(intakePreviouslyDeployed) {
                new SetIntakeCommand(intake, IntakeState.Retracted).schedule();
                intakePreviouslyDeployed = false;
//                LEDs.getInstance().setState(LEDs.State.Yellow);
            } else {
                new SetIntakeCommand(intake, IntakeState.Running).schedule();
                intakePreviouslyDeployed = true;
//                LEDs.getInstance().enableDefaultState();
            }
        }
        intakeButtonPreviouslyPressed = ControlsManager.getButton(Control.IntakeButton);
    }

    @Override
    public void end() {
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, 0));
    }
}
