package com.team2137.frc2021.commands;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrajectoryFollowCommand extends CommandBase {

    private HolonomicDriveController holonomicController;
    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController thetaController;
    private Rotation2d currentHeadingTarget;

    private Timer timer;

    private Trajectory trajectory;

    private SwerveDrivetrain drivetrain;

    private Rotation2d desiredFinalHeading;

    private Rotation2d initialHeading;
    private HeadingControlThreshold[] thresholds;
    private LineSide previousSide;
    private int thresholdIndex = 0;

    private TrajectoryMode trajectoryMode;

    /**
     * Creates a trajectory follower for swerve with multiple heading targets
     * @param drivetrain the drivetrain object
     * @param trajectory the trajectory to follow - heading doesn't matter
     * @param initialHeadingTarget the initial heading to target
     * @param thresholds an array of thresholds - target is used AFTER passing the line
     */
    public TrajectoryFollowCommand(SwerveDrivetrain drivetrain, Trajectory trajectory, Rotation2d initialHeadingTarget, HeadingControlThreshold... thresholds) {
        timer = new Timer();

        this.trajectory = trajectory;

        Constants.PIDConstants translationConstants = Constants.Drivetrain.translationPIDConstants;
        this.xController = new PIDController(translationConstants.P, translationConstants.I, translationConstants.D);
        this.yController = new PIDController(translationConstants.P, translationConstants.I, translationConstants.D);

        Constants.PIDConstants thetaConstants = Constants.Drivetrain.thetaPIDConstants;
        TrapezoidProfile.Constraints thetaConstraints = Constants.Drivetrain.thetaPIDConstraints;
        this.thetaController = new ProfiledPIDController(thetaConstants.P, thetaConstants.I, thetaConstants.D, thetaConstraints);

        holonomicController = new HolonomicDriveController(xController, yController, thetaController);

        this.initialHeading = initialHeadingTarget;
        this.thresholds = thresholds;
        this.currentHeadingTarget = initialHeadingTarget;

        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    /**
     * Creates a trajectory follower for swerve without multiple heading targets
     * @param drivetrain the drivetrain object
     * @param trajectory the trajectory to follow - heading doesn't matter
     * @param headingTarget the heading to target
     */
    public TrajectoryFollowCommand(SwerveDrivetrain drivetrain, Trajectory trajectory, Rotation2d headingTarget) {
        timer = new Timer();

        this.trajectory = trajectory;

        Constants.PIDConstants translationConstants = Constants.Drivetrain.translationPIDConstants;
        this.xController = new PIDController(translationConstants.P, translationConstants.I, translationConstants.D);
        this.yController = new PIDController(translationConstants.P, translationConstants.I, translationConstants.D);

        Constants.PIDConstants thetaConstants = Constants.Drivetrain.thetaPIDConstants;
        TrapezoidProfile.Constraints thetaConstraints = Constants.Drivetrain.thetaPIDConstraints;
        this.thetaController = new ProfiledPIDController(thetaConstants.P, thetaConstants.I, thetaConstants.D, thetaConstraints);

        holonomicController = new HolonomicDriveController(xController, yController, thetaController);

        this.trajectoryMode = TrajectoryMode.SingleHeading;
        this.desiredFinalHeading = headingTarget;

        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if(trajectoryMode == TrajectoryMode.MultiHeading) {
            previousSide = thresholds[0].getLineSide(drivetrain.getPose().getTranslation());
        }
    }

    @Override
    public void execute() {
        var desiredState = trajectory.sample(timer.get());

        switch(trajectoryMode) {
            case SingleHeading:
                currentHeadingTarget = desiredFinalHeading;
                break;
            case MultiHeading:
                if(thresholdIndex < thresholds.length) {
                    var currentThreshold = thresholds[thresholdIndex];
                    var currentSide = currentThreshold.getLineSide(drivetrain.getPose().getTranslation());
                    if (currentSide != previousSide) {
                        currentHeadingTarget = currentThreshold.getTarget();
                        thresholdIndex++;
                        previousSide = thresholds[thresholdIndex].getLineSide(drivetrain.getPose().getTranslation());
                    }
                }
                break;
        }

        var target_speeds = holonomicController.calculate(drivetrain.getPose(), desiredState, currentHeadingTarget);

        drivetrain.driveTranslationRotationVelocity(target_speeds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public class HeadingControlThreshold {
        private double m;
        private double b;
        private Rotation2d target;

        /**
         * @param m the constant m of y=mx+b
         * @param b the constant b of y=mx+b
         * @param target the angle to turn to after passing this line
         */
        public HeadingControlThreshold(double m, double b, Rotation2d target) {
            this.m = m;
            this.b = b;
            this.target = target;
        }

        /**
         * @param position the position of the robot
         * @return the side of the line that the robot is on
         */
        public LineSide getLineSide(Translation2d position) {
            double lineY = m * position.getX() + b;
            return position.getY() - lineY >= 0 ? LineSide.Positive : LineSide.Negative;
        }

        /**
         * @return the target for after the line is passed
         */
        public Rotation2d getTarget() {
            return target;
        }
    }

    public enum LineSide {
        Negative, Positive
    }

    private enum TrajectoryMode {
        SingleHeading, MultiHeading
    }
}
