package frc.robot.hardware;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import frc.robot.util.Encoder;
import frc.robot.util.Motor;
import frc.robot.util.PID;

import javax.annotation.Nullable;


public class SwerveModule {

    private CANSparkMax mDriveMotor;
    private CANSparkMax mTurnMotor;
    private CANCoder mCANCoder;

    private CANPIDController mDrivePIDController;
    private CANPIDController mTurnPIDController;

    /**
     * Creates a SwerveModule that contains the Motors and encoder for a 2 Motor Swerve Module
     * @param driveMotor - Pass a Motor object and inside the class it is turned to a CANSpark Max
     * @param turnMotor - Pass a Motor object and inside the class it is turned to a CANSpark Max
     * @param turnEncoder - Pass a Encoder object and indide the class it is turn to a CANCoder
     */
    public SwerveModule (Motor driveMotor, PID drivePID, Motor turnMotor, PID turnPID, Encoder turnEncoder) {
        this(driveMotor, turnMotor, turnEncoder);

        mDrivePIDController = mDriveMotor.getPIDController();
        mTurnPIDController = mTurnMotor.getPIDController();

        mDrivePIDController.setP(drivePID.getP());
        mDrivePIDController.setI(drivePID.getI());
        mDrivePIDController.setD(drivePID.getD());

        mTurnPIDController.setP(turnPID.getP());
        mTurnPIDController.setI(turnPID.getI());
        mTurnPIDController.setD(turnPID.getD());

    }

    public SwerveModule (Motor driveMotor, Motor turnMotor, Encoder turnEncoder) {
        mDriveMotor = new CANSparkMax(driveMotor.getMotorID(), CANSparkMaxLowLevel.MotorType.kBrushless);
        mTurnMotor = new CANSparkMax(turnMotor.getMotorID(), CANSparkMaxLowLevel.MotorType.kBrushless);

        //Reset the motors to the default state
        mDriveMotor.restoreFactoryDefaults();
        mTurnMotor.restoreFactoryDefaults();

        //Set the right inversion
        mDriveMotor.setInverted(driveMotor.inverted());
        mTurnMotor.setInverted(turnMotor.inverted());

        //Turn the motors to Braking
        mDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        //Add a current limit to stop from buring out the motor and controller
        mDriveMotor.setSmartCurrentLimit(driveMotor.getCurrentLimit());
        mTurnMotor.setSmartCurrentLimit(turnMotor.getCurrentLimit());

        mCANCoder = new CANCoder(turnEncoder.getEncoderID());
    }

    public void setWheelDegree(double pos) {
        this.mTurnPIDController.setReference(pos, ControlType.kPosition);
    }
}
