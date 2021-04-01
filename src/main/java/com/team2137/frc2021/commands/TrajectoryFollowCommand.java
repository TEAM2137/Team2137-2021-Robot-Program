package com.team2137.frc2021.commands;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import com.team2137.frc2021.util.PID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private ThresholdType currentType = ThresholdType.Static;
    private HeadingControlThreshold activeThreshold;

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

        PID translationConstants = Constants.Drivetrain.translationPIDConstants;
        this.xController = new PIDController(translationConstants.getP(), translationConstants.getI(), translationConstants.getD());
        this.yController = new PIDController(translationConstants.getP(), translationConstants.getI(), translationConstants.getD());

        PID thetaConstants = Constants.Drivetrain.autoThetaPIDConstants;
        TrapezoidProfile.Constraints thetaConstraints = Constants.Drivetrain.autoThetaPIDConstraints;
        this.thetaController = new ProfiledPIDController(thetaConstants.getP(), thetaConstants.getI(), thetaConstants.getD(), thetaConstraints);

        holonomicController = new HolonomicDriveController(xController, yController, thetaController);

        this.initialHeading = initialHeadingTarget;
        this.thresholds = thresholds;
        this.currentHeadingTarget = initialHeadingTarget;

        this.trajectoryMode = TrajectoryMode.MultiHeading;

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

        PID translationConstants = Constants.Drivetrain.translationPIDConstants;
        this.xController = new PIDController(translationConstants.getP(), translationConstants.getI(), translationConstants.getD());
        this.yController = new PIDController(translationConstants.getP(), translationConstants.getI(), translationConstants.getD());

        PID thetaConstants = Constants.Drivetrain.autoThetaPIDConstants;
        TrapezoidProfile.Constraints thetaConstraints = Constants.Drivetrain.autoThetaPIDConstraints;
        this.thetaController = new ProfiledPIDController(thetaConstants.getP(), thetaConstants.getI(), thetaConstants.getD(), thetaConstraints);

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
                        currentType = currentThreshold.type;
                        activeThreshold = currentThreshold;
                        thresholdIndex++;
                        if(thresholdIndex < thresholds.length) {
                            previousSide = thresholds[thresholdIndex].getLineSide(drivetrain.getPose().getTranslation());
                        }
                    }

                    if (currentType == ThresholdType.Dynamic) {
                        currentHeadingTarget = desiredState.poseMeters.getRotation().plus(activeThreshold.getOffset());
                    }
                    SmartDashboard.putString("side", currentSide.toString());
                }
                SmartDashboard.putNumber("current threshold", thresholdIndex);
                break;
        }

        var target_speeds = holonomicController.calculate(drivetrain.getPose(), desiredState, currentHeadingTarget);
        SmartDashboard.putNumber("heading target", currentHeadingTarget.getDegrees());

        drivetrain.driveTranslationRotationVelocity(target_speeds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drivetrain.setAllModuleDriveRawPower(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public static class HeadingControlThreshold {
        private double m;
        private double b;
        private Rotation2d target;
        private Rotation2d offset;
        public ThresholdType type;

        /**
         * @param m the constant m of y=mx+b
         * @param b the constant b of y=mx+b
         * @param target the angle to turn to after passing this line
         */
        public HeadingControlThreshold(double m, double b, Rotation2d target) {
            this.m = m;
            this.b = b;
            this.target = target;
            this.type = ThresholdType.Static;
        }

        /**
         * @param point1 the first point of the threshold line
         * @param point2 the second point of the threshold line
         * @param value the angle to turn to after passing this line
         * @param type the type of heading threshold
         */
        public HeadingControlThreshold(Translation2d point1, Translation2d point2, Rotation2d value, ThresholdType type) {
            // prevent a vertical line with an infinite/undefined slope, divide by zero error
            if(point1.getX() == point2.getX()) {
                point1 = point1.plus(new Translation2d(0.0001, 0));
            }

            //rise over run
            this.m = (point1.getY() - point2.getY()) / (point1.getX() - point2.getX());
            //determine y-intercept by rearranging y=mx+b
            this.b = point2.getY() - (this.m * point2.getX());

            if (type == ThresholdType.Static) {
                this.target = value;
                this.type = ThresholdType.Static;
            } else if (type == ThresholdType.Dynamic) {
                this.offset = value;
                this.type = ThresholdType.Dynamic;
            }
        }

        /**
         * @param position the position of the robot
         * @return the side of the line that the robot is on
         */
        public LineSide getLineSide(Translation2d position) {
            double lineY = m * position.getX() + b;
            SmartDashboard.putNumber("liney", lineY);
            return position.getY() >= lineY ? LineSide.Positive : LineSide.Negative;
        }

        /**
         * @return the target for after the line is passed
         */
        public Rotation2d getTarget() {
            return target;
        }

        /**
         * @return the heading offset for dynamic targets
         */
        public Rotation2d getOffset() {
            return target;
        }
    }

    public enum ThresholdType {
        /**
         * Where the heading target for the robot is static and doesn't change, it simply targets a single value
         */
        Static,
        /**
         * Where the heading target is based off of the trajectory's heading, plus an offset
         */
        Dynamic
    }

    public enum LineSide {
        Negative, Positive
    }

    private enum TrajectoryMode {
        SingleHeading, MultiHeading
    }
}
