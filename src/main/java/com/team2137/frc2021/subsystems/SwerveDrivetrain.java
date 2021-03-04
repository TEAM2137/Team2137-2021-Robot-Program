package com.team2137.frc2021.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team2137.frc2021.Constants;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;

// Everything in this file will be done in the order front left, front right, back left, back right
public class SwerveDrivetrain extends SubsystemBase {

    SwerveDriveKinematics kinematics;

    private SwerveDriveModule frontLeftModule;
    private SwerveDriveModule frontRightModule;
    private SwerveDriveModule backLeftModule;
    private SwerveDriveModule backRightModule;

    private SwerveDriveModule[] swerveArray;

    private PigeonIMU pigeonIMU;

    private SwerveDrivePoseEstimator poseEstimator;

    private Field2d field2d = new Field2d();

    /**
     * Creates a swerve drivetrain (uses values from constants)
     */
    public SwerveDrivetrain() {
        // locations of all of the modules (for kinematics)
        Translation2d frontLeftLocation = new Translation2d(Constants.Drivetrain.length/2, Constants.Drivetrain.width/2);
        Translation2d frontRightLocation = new Translation2d(Constants.Drivetrain.length/2, -Constants.Drivetrain.width/2);
        Translation2d backLeftLocation = new Translation2d(-Constants.Drivetrain.length/2, Constants.Drivetrain.width/2);
        Translation2d backRightLocation = new Translation2d(-Constants.Drivetrain.length/2, -Constants.Drivetrain.width/2);

        // the kinematics object for converting chassis speeds to module rotations and powers
        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        // each of the modules
        frontLeftModule = new SwerveDriveModule(Constants.Drivetrain.frontLeft);
        frontRightModule = new SwerveDriveModule(Constants.Drivetrain.frontRight);
        backLeftModule = new SwerveDriveModule(Constants.Drivetrain.backLeft);
        backRightModule = new SwerveDriveModule(Constants.Drivetrain.backRight);

        // an array of the swerve modules, to make life easier
        swerveArray = new SwerveDriveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        // the gyro
        pigeonIMU = new PigeonIMU(Constants.Drivetrain.gyroID);
        pigeonIMU.configFactoryDefault();
        calibrateGyro();
        pigeonIMU.setYaw(0);

        // create pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(new Rotation2d(), new Pose2d(), kinematics,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, Units.degreesToRadians(5)), // State measurement standard deviations. X, Y, theta.
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Units.degreesToRadians(0.01)), // Local measurement standard deviations. Gyro.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.5, 0.5, Units.degreesToRadians(30))); // Vision measurement standard deviations. X, Y, and theta.

        SmartDashboard.putData("Field", field2d);
        SmartDashboard.putBoolean("Reset Position", false);
    }

    /**
     * Periodic loop of the subsystem
     */
    @Override
    public void periodic() {
        poseEstimator.update(getRobotAngle(), getSwerveModuleStates());

        SmartDashboard.putNumber("Drivetrain Angle", getRobotAngle().getDegrees());
        field2d.setRobotPose(getPose());
        SmartDashboard.putNumber("Drivetrain X", Units.metersToFeet(getPose().getX()));
        SmartDashboard.putNumber("Drivetrain Y", Units.metersToFeet(getPose().getY()));

        if(SmartDashboard.getBoolean("Reset Position", false)) {
//            resetOdometry();
            SmartDashboard.putBoolean("Reset Position", false);
        }

        for(SwerveDriveModule module : swerveArray) {
            module.periodic();
        }
    }

    /**
     * @return the angle of the robot (CCW positive (normal))
     */
    public Rotation2d getRobotAngle() {
        double[] ypr = new double[3];
        pigeonIMU.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]).minus(new Rotation2d());
    }

    public void calibrateGyro() {
        pigeonIMU.enterCalibrationMode(PigeonIMU.CalibrationMode.Temperature);
    }

    public void addVisionReading(Pose2d pose, double processingTime) {
        this.poseEstimator.addVisionMeasurement(pose, processingTime);
    }

    /**
     * @param speeds speed of the chassis with -1 to 1 on translation
     */
    public void driveTranslationRotationRaw(ChassisSpeeds speeds) {
        if(speeds.vxMetersPerSecond + speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond == 0) {
            // if power isn't being applied, don't set the module rotation to zero
            for (int i = 0; i < swerveArray.length; i++) {
                swerveArray[i].setDrivePowerRaw(0);
                swerveArray[i].setTurningTarget(swerveArray[i].getModuleRotation());
            }
        } else {
            // if power, drive it
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds); //convert speeds to individual modules

            SwerveDriveKinematics.normalizeWheelSpeeds(states, 1); //normalize speeds to be all between -1 and 1
            for (int i = 0; i < states.length; i++) {
                //optimize module rotation (instead of a >90 degree turn, turn less and flip wheel direction)
                states[i] = SwerveModuleState.optimize(states[i], swerveArray[i].getModuleRotation());

                //set all the things
                swerveArray[i].setTurningTarget(states[i].angle);
                swerveArray[i].setDrivePowerRaw(states[i].speedMetersPerSecond);
            }
        }
    }

    /**
     * @param speeds speed of the chassis in m/s and rad/s
     */
    public void driveTranslationRotationVelocity(ChassisSpeeds speeds) {
        if(speeds.vxMetersPerSecond + speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond == 0) {
            // if power isn't being applied, don't set the module rotation to zero
            for (int i = 0; i < swerveArray.length; i++) {
                swerveArray[i].setDriveVelocity(0);
                swerveArray[i].setTurningTarget(swerveArray[i].getModuleRotation());
            }
        } else {
            // if power, drive it
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds); //convert speeds to individual modules

            SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.Drivetrain.driveMaxSpeed); //normalize speeds to be all between min and max speed
            for (int i = 0; i < states.length; i++) {
                //optimize module rotation (instead of a >90 degree turn, turn less and flip wheel direction)
                states[i] = SwerveModuleState.optimize(states[i], swerveArray[i].getModuleRotation());

                //set all the things
                swerveArray[i].setTurningTarget(states[i].angle);
                swerveArray[i].setDriveVelocity(states[i].speedMetersPerSecond);
            }
        }
    }

    /**
     * @return swerve module states in m/s
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < swerveArray.length; i++) {
            states[i] = swerveArray[i].getSwerveModuleState();
        }
        return states;
    }

    /**
     * @return the pose of the robot in meters
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * @param measurement Pose2d of the calculated position
     * @param timestamp the timestamp the measurement is from
     */
    public void addVisionMeasurement(Pose2d measurement, double timestamp) {
        poseEstimator.addVisionMeasurement(measurement, timestamp);
    }

    /**
     * For debugging
     * @param power power of the drive motors -1 to 1
     */
    public void setAllModuleDriveRawPower(double power) {
        frontLeftModule.setDrivePowerRaw(power);
        frontRightModule.setDrivePowerRaw(power);
        backLeftModule.setDrivePowerRaw(power);
        backRightModule.setDrivePowerRaw(power);
    }

    /**
     * For debugging
     * @param angle angle of all of the modules
     */
    public void setAllModuleRotations(Rotation2d angle) {
        frontLeftModule.setTurningTarget(angle);
        frontRightModule.setTurningTarget(angle);
        backLeftModule.setTurningTarget(angle);
        backRightModule.setTurningTarget(angle);
    }

    /**
     * For debugging
     * @param velocity target velocity in m/s
     */
    public void setAllModuleDriveVelocity(double velocity) {
        frontLeftModule.setDriveVelocity(velocity);
        frontRightModule.setDriveVelocity(velocity);
        backLeftModule.setDriveVelocity(velocity);
        backRightModule.setDriveVelocity(velocity);
    }

    public void selfTargetAllModuleAngles() {
        frontLeftModule.selfTargetAngle();
        frontRightModule.selfTargetAngle();
        backLeftModule.selfTargetAngle();
        backRightModule.selfTargetAngle();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(pose, pose.getRotation());
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    /**
     * Sets the drivetrain into x-locking mode, making it defense resistant
     */
    public void xLock() {
        double length = Constants.Drivetrain.length / 2;
        double width = Constants.Drivetrain.width / 2;
        setAllModuleDriveRawPower(0);

        frontLeftModule.setTurningTarget(new Rotation2d(Math.atan2(width, length)));
        frontRightModule.setTurningTarget(new Rotation2d(Math.atan2(-width, length)));
        backLeftModule.setTurningTarget(new Rotation2d(Math.atan2(width, -length)));
        backRightModule.setTurningTarget(new Rotation2d(Math.atan2(-width, -length)));
    }

    public TrajectoryConfig getDefaultConstraint() {
        return new TrajectoryConfig(Constants.Drivetrain.driveMaxSpeed, Constants.Drivetrain.driveMaxAccel).setKinematics(kinematics);
    }
}
