package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.commands.SetIntakeCommand;
import com.team2137.frc2021.program.ControlsManager.Control;
import com.team2137.frc2021.subsystems.LEDs;
import com.team2137.frc2021.util.PID;
import com.team2137.libs.UnitsExtra;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import com.team2137.frc2021.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

        LEDs.getInstance().setDefaultState(LEDs.State.Blue, true);
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
            Translation2d target = UnitsExtra.feetToMeters(new Translation2d(0, 0));

            //Rotation2d angle = new Rotation2d(-(drivetrain.getPose().getX() - target.getX()), -(drivetrain.getPose().getY() - target.getY()));
            Rotation2d angle;
            if (limeLight.hasTarget())
                angle = new Rotation2d(-(limeLight.translateCameraXToCenter(drivetrain.getRobotAngle().getDegrees()) - target.getX()), -(limeLight.translateCameraYToCenter(drivetrain.getRobotAngle().getDegrees()) - target.getY()));
            else
                angle = new Rotation2d(-(drivetrain.getPose().getX() - target.getX()), -(drivetrain.getPose().getY() - target.getY()));

            double thetaPower = headingController.calculate(drivetrain.getRobotAngle().getRadians(), angle.getRadians());

            if(Math.abs(thetaPower) < 0.05) {
                thetaPower = 0;
            }

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe,
                    thetaPower,
                    drivetrain.getRobotAngle());

            drivetrain.driveTranslationRotationRaw(speeds);

        } else {
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
                LEDs.getInstance().setState(LEDs.State.Yellow);
            } else {
                new SetIntakeCommand(intake, IntakeState.Running).schedule();
                intakePreviouslyDeployed = true;
                LEDs.getInstance().enableDefaultState();
            }
        }
        intakeButtonPreviouslyPressed = ControlsManager.getButton(Control.IntakeButton);
    }

    @Override
    public void end() {
        drivetrain.driveTranslationRotationRaw(new ChassisSpeeds(0, 0, 0));
    }
}
