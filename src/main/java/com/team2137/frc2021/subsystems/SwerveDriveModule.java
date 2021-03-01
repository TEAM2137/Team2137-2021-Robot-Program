package com.team2137.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.team2137.frc2021.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import com.team2137.frc2021.util.PID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveModule extends SubsystemBase {

    private TalonFX driveMotor;
    private TalonFX turningMotor;
    private CANCoder encoder;

    private PIDController turningPID;

    private double encoderOffset;

    private Rotation2d turningSetpointRaw = Rotation2d.fromDegrees(0);
    private Rotation2d turningSetpointCorrected = Rotation2d.fromDegrees(0);
    private boolean reverseWheel;

    private double driveRawPower = 0;
    private double driveVelocityTarget;
    private PIDController drivePID;
    private SimpleMotorFeedforward driveFeedForward;
    private DriveMode driveMode = DriveMode.RawPower;

    public final String moduleName;

    /**
     * Creats a swerve module
     * @param driveID CAN id of the drive motor
     * @param turningID CAN id of the turning motor
     * @param encoderID CAN id of the encoder
     * @param encoderOffset offset of the encoder on module rotation (wheels forward, bevels left)
     * @param moduleName name of the module for debug purposes
     */
    public SwerveDriveModule(int driveID, int turningID, int encoderID, double encoderOffset, String moduleName) {
        // Drive motor setup
        this.driveMotor = new TalonFX(driveID);
        this.driveMotor.configFactoryDefault();
        this.driveMotor.setInverted(Constants.Drivetrain.invertDriveMotor);
        this.driveMotor.configSupplyCurrentLimit(Constants.Drivetrain.driveMotorSupplyCurrentLimit);
        this.driveMotor.configStatorCurrentLimit(Constants.Drivetrain.driveMotorStatorCurrentLimit);
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.driveMotor.configClosedloopRamp(Constants.Drivetrain.driveMotorRamp);
        this.driveMotor.configOpenloopRamp(Constants.Drivetrain.driveMotorRamp);

        // Turning motor setup
        this.turningMotor = new TalonFX(turningID);
        this.turningMotor.configFactoryDefault();
        this.turningMotor.setInverted(Constants.Drivetrain.invertTurningMotor);
        this.turningMotor.configSupplyCurrentLimit(Constants.Drivetrain.turningMotorSupplyCurrentLimit);
        this.turningMotor.configStatorCurrentLimit(Constants.Drivetrain.turningMotorStatorCurrentLimit);
        this.turningMotor.setNeutralMode(NeutralMode.Brake);

        // Encoder setup
        this.encoder = new CANCoder(encoderID);
        this.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.encoder.configMagnetOffset(encoderOffset);

        this.encoderOffset = encoderOffset;

        this.moduleName = moduleName;

        // Setup turning pid
        PID turningPIDConstants = Constants.Drivetrain.turningPIDConstants;
        this.turningPID = new PIDController(turningPIDConstants.getP(), turningPIDConstants.getI(), turningPIDConstants.getD());
        this.turningPID.enableContinuousInput(-180, 180); // allows module to wrap properly

        // Setup drive pid
        PID drivePIDConstants = Constants.Drivetrain.drivePIDConstants;
        this.drivePID = new PIDController(drivePIDConstants.getP(), drivePIDConstants.getI(), drivePIDConstants.getD());
        this.driveFeedForward = Constants.Drivetrain.driveFeedforward;

        this.selfTargetAngle();


    }

    /**
     * Creates a swerve module
     * @param constants the swerve module to create
     */
    public SwerveDriveModule(Constants.Drivetrain.SwerveModuleConstants constants) {
        this(constants.driveID, constants.turningID, constants.encoderID, constants.offset, constants.moduleName);
    }

    /**
     * Periodic loop of the subsystem
     */
    @Override
    public void periodic() {
        // if robot is disabled, target modules to their current angle
        // if you're doing some types of debugging, disable this
        if(DriverStation.getInstance().isDisabled()) {
//            selfTargetAngle();
        }

        // prevents the module from doing a >90 degree flip to get to a target, instead reverse wheel direction

        //optimization now done in SwerveDrivetrain's methods, so this is obsolete
        if (Math.abs(turningSetpointRaw.minus(getModuleRotation()).getDegrees()) > 90) {
            turningSetpointCorrected = turningSetpointRaw.plus(Rotation2d.fromDegrees(180));
            reverseWheel = true;
        } else {
            turningSetpointCorrected = turningSetpointRaw;
            reverseWheel = false;
        }

//        double output = turningPID.calculate(getModuleRotation().getDegrees(), turningSetpointRaw.getDegrees());
        double pidEffort = turningPID.calculate(getModuleRotation().getDegrees(), turningSetpointCorrected.getDegrees());

        double output = Math.signum(pidEffort) * Constants.Drivetrain.turningFeedForward + pidEffort;

        if (moduleName == "Front Right") {
            System.out.println(moduleName + "setcor" + turningSetpointCorrected.getDegrees());
            System.out.println("modrot" + getModuleRotation().getDegrees());
            System.out.println("pid" + pidEffort);
            System.out.println(output);
        }

        turningMotor.set(ControlMode.PercentOutput, output / 12);

        switch(driveMode) {
            case RawPower: //for use in teleop
                driveMotor.set(ControlMode.PercentOutput, driveRawPower * (reverseWheel ? -1 : 1));
//                driveMotor.set(driveRawPower);
                break;
            case Velocity: //for use in auto and autonomous trajectories
//                driveMotor.setVoltage(driveFeedForward.calculate(driveVelocityTarget * (reverseWheel ? -1 : 1)) +
//                        drivePIDController.calculate(getDriveVelocity(), driveVelocityTarget * (reverseWheel ? -1 : 1)));

                driveMotor.set(ControlMode.PercentOutput, (driveFeedForward.calculate(driveVelocityTarget) +
                        drivePID.calculate(getDriveVelocity(), driveVelocityTarget)) / 12);
                break;
        }

        SmartDashboard.putNumber(moduleName + " Heading Position", getModuleRotation().getDegrees());
        SmartDashboard.putNumber(moduleName + " Heading Target", turningSetpointCorrected.getDegrees());
        SmartDashboard.putNumber(moduleName + " Heading Error", turningPID.getPositionError());
        SmartDashboard.putNumber(moduleName + " Heading Power", turningMotor.getMotorOutputPercent());

        SmartDashboard.putNumber(moduleName + " Drive Power", driveMotor.getMotorOutputPercent());

        SmartDashboard.putNumber(moduleName + " Velocity Target", Units.metersToFeet(driveVelocityTarget));
        SmartDashboard.putNumber(moduleName + " Velocity", Units.metersToFeet(getDriveVelocity()));

    }

    /**
     * @return Rotation2d with the rotation of the module direct from encoder (not dealing with optimization)
     */
    public Rotation2d getModuleRotation() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }

    /**
     * @param target Rotation2d with the target angle, unoptimized
     */
    public void setTurningTarget(Rotation2d target) {
        turningSetpointRaw = target;
    }

    /**
     * @param power from -1 to 1
     */
    public void setDrivePowerRaw(double power) {
        driveRawPower = power;
        driveMode = DriveMode.RawPower;
    }


    /**
     * @param velocity in meters per second
     */
    public void setDriveVelocity(double velocity) {
        driveVelocityTarget = velocity;
        driveMode = DriveMode.Velocity;
    }

    /**
     * @return Drive wheel velocity in meters per second
     */
    public double getDriveVelocity() {
        return driveMotor.getSensorCollection().getIntegratedSensorVelocity() * Constants.Drivetrain.motorToWheelConversionFactor / 2048 * 10;
    }

    /**
     * @return wheel distance traveled in meters
     */
    public double getDriveDistance() {
        return driveMotor.getSensorCollection().getIntegratedSensorPosition() * Constants.Drivetrain.motorToWheelConversionFactor / 2048;
    }

    /**
     * Resets drive encoder distance to zero.
     */
    public void resetDriveEncoder() {
        driveMotor.getSensorCollection().setIntegratedSensorPosition(0, 10);
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getModuleRotation());
    }

    public void selfTargetAngle() {
        setTurningTarget(getModuleRotation());
    }

    private enum DriveMode {
        RawPower, Velocity
    }
}
