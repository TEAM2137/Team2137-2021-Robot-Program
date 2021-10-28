package com.team2137.frc2021.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.team2137.frc2021.Constants;
import edu.wpi.first.wpilibj.DriverStation;
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

import static com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode.BootTareGyroAccel;

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

    private Rotation2d gyroOffset = new Rotation2d();

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
        pigeonIMU.setFusedHeading(0);

        // create pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(new Rotation2d(), new Pose2d(), kinematics,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, Units.degreesToRadians(5)), // State measurement standard deviations. X, Y, theta.
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Units.degreesToRadians(0.01)), // Local measurement standard deviations. Gyro.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.5, 0.5, Units.degreesToRadians(30))); // Vision measurement standard deviations. X, Y, and theta.

        SmartDashboard.putData("Field", field2d);
//        SmartDashboard.putBoolean("Reset Position", false);

        resetOdometry();
    }

    /**
     * Periodic loop of the subsystem
     */
    @Override
    public void periodic() {
        poseEstimator.update(getRobotAngle().plus(gyroOffset), getSwerveModuleStates());
//        poseEstimator.update(getRobotAngle(), getSwerveModuleStates());

        field2d.setRobotPose(getPose());

        SmartDashboard.putNumber("Drivetrain Angle", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Drivetrain X", Units.metersToFeet(getPose().getX()));
        SmartDashboard.putNumber("Drivetrain Y", Units.metersToFeet(getPose().getY()));

//        if(SmartDashboard.getBoolean("Reset Position", false)) {
//            resetOdometry();
//            SmartDashboard.putBoolean("Reset Position", false);
//        }

        for(SwerveDriveModule module : swerveArray) {
            module.periodic();
        }

        SmartDashboard.putNumber("FusedHeading", getRobotAngle().getDegrees());
    }

    /**
     * @return the angle of the robot (CCW positive (normal))
     */
    public Rotation2d getRobotAngle() {
        double raw = pigeonIMU.getFusedHeading(); // % 360;
        while (raw <= -180) raw += 360;
        while (raw > 180) raw -= 360;
        return Rotation2d.fromDegrees(raw);

//        if (raw < 0 && raw > -180) {
//            return Rotation2d.fromDegrees()
//        }
//        if( raw > 180) {
//            raw = raw - 360;
//            pigeonIMU.setFusedHeading(raw);
//        }else if( raw < -180) {
//            raw = raw + 360;
//            pigeonIMU.setFusedHeading(raw);
//        }
//        return Rotation2d.fromDegrees(raw);
//        if (value > 180)
//            return Rotation2d.fromDegrees(value - 360);
//        else
//            return Rotation2d.fromDegrees(value);
//        return Rotation2d.fromDegrees(Math.abs(pigeonIMU.getFusedHeading()) % 360);
    }

    public double getThetaVelocity() {
        double[] tmp = new double[3];
        pigeonIMU.getRawGyro(tmp);
        return tmp[2];
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
        var initialPose = poseEstimator.getEstimatedPosition();

        return new Pose2d(-initialPose.getX(), -initialPose.getY(), initialPose.getRotation());
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
        gyroOffset = pose.getRotation().minus(getRobotAngle());
//        poseEstimator.resetPosition(new Pose2d(-pose.getX(), -pose.getY(), pose.getRotation()), pose.getRotation());
        poseEstimator.resetPosition(new Pose2d(-pose.getX(), -pose.getY(), pose.getRotation()), getRobotAngle());
//        poseEstimator.resetPosition(new Pose2d(-pose.getX(), -pose.getY(), Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(0));
//        DriverStation.reportError("Resetting pose: X: " + -pose.getX() + " Y: " + -pose.getY(), false);
//        DriverStation.reportError("Rotation: " + pose.getRotation().getDegrees(), false);
//        DriverStation.reportError("Measured: " + getPose().getRotation().getDegrees(), false);
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
//        return new TrajectoryConfig(Units.feetToMeters(13), Constants.Drivetrain.driveMaxAccel).s
//        etKinematics(kinematics);
//        return new TrajectoryConfig(Units.feetToMeters(13), Constants.Drivetrain.driveMaxAccel);
//        return new TrajectoryConfig(Constants.Drivetrain.driveMaxSpeed, Constants.Drivetrain.driveMaxAccel);
    }

    public void setBrakeMode(boolean brake) {
        this.frontLeftModule.setDriveMode(brake);
        this.backLeftModule.setDriveMode(brake);
        this.frontRightModule.setDriveMode(brake);
        this.backRightModule.setDriveMode(brake);
    }
}
