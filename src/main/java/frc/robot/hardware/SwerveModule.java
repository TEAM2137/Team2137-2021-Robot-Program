package frc.robot.hardware;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import frc.robot.base.Hardware;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.util.Constants;
import frc.robot.util.Encoder;
import frc.robot.util.Motor;
import frc.robot.util.PID;

import javax.annotation.Nullable;

public class SwerveModule {

    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private double dblDriveMotorStartPos = 0;
    private double dblTurnMotorStartPos = 0;

    private double dblDriveMotorConversionFactor = 0;
    private double dblTurnMotorConversionFactor = 0;

    private CANPIDController drivePID;
    private CANPIDController turnPID;

    private CANEncoder driveEncoder;
    private CANEncoder primaryTurnEncoder;
    private CANCoder secondaryTurnEncoder;

    private int turnMotorOffset = 0;

    private boolean PIDEnabledDrive = true;

    public SwerveModule(Motor drive, Motor turn, Encoder secondaryTurn) {
        this.driveMotor = new CANSparkMax(drive.getMotorID(), drive.getMotorType().getREVType());
        this.turnMotor = new CANSparkMax(turn.getMotorID(), turn.getMotorType().getREVType());

        this.driveMotor.restoreFactoryDefaults();
        this.driveMotor.setInverted(drive.inverted());
        this.driveMotor.setSmartCurrentLimit(drive.getCurrentLimit());
        this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.turnMotor.restoreFactoryDefaults();
        this.turnMotor.setInverted(turn.inverted());
        this.turnMotor.setSmartCurrentLimit(turn.getCurrentLimit());
        this.turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.driveEncoder = driveMotor.getEncoder();
        this.primaryTurnEncoder = turnMotor.getEncoder();
        this.secondaryTurnEncoder = new CANCoder(secondaryTurn.getEncoderID());

        this.turnMotorOffset = Integer.parseInt(secondaryTurn.getParm(0));

        this.secondaryTurnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.secondaryTurnEncoder.configMagnetOffset(turnMotorOffset);

        this.dblDriveMotorConversionFactor = drive.getGearRatio();
        this.dblTurnMotorConversionFactor = turn.getGearRatio();

        dblDriveMotorStartPos = this.driveEncoder.getPosition();
        dblTurnMotorStartPos = this.primaryTurnEncoder.getPosition();
    }

    public SwerveModule(Motor drive, @Nullable PID drivePID, Motor turn, PID turnPID, Encoder secondaryTurn) {
        this(drive, turn, secondaryTurn);
        PIDEnabledDrive = true;

        if (drivePID != null) this.drivePID = this.driveMotor.getPIDController();
        this.turnPID = this.turnMotor.getPIDController();

        if (drivePID != null) {
            this.drivePID.setP(drivePID.getP());
            this.drivePID.setI(drivePID.getI());
            this.drivePID.setD(drivePID.getD());
        }

        this.turnPID.setP(turnPID.getP());
        this.turnPID.setI(turnPID.getI());
        this.turnPID.setD(turnPID.getD());
    }

    public double getTurnMotorPosition() {
        return (((this.primaryTurnEncoder.getPosition() - dblTurnMotorStartPos) * this.dblTurnMotorConversionFactor) / 360);
    }

    public void setTurnMotorPosition(double angle) {
        Rotation2d encoderPosition = Rotation2d.fromDegrees(this.secondaryTurnEncoder.getAbsolutePosition() - this.turnMotorOffset);
        boolean reverseWheel = false;

        if (Math.abs(encoderPosition.minus(Rotation2d.fromDegrees(angle)).getDegrees()) > 90) {
            encoderPosition = encoderPosition.plus(Rotation2d.fromDegrees(180));
            reverseWheel = true;
        }

        if (PIDEnabledDrive) {
            double setPoint = (Math.abs(encoderPosition.minus(Rotation2d.fromDegrees(angle)).getDegrees()) * (reverseWheel ? -1 : 1));
            this.turnPID.setReference(primaryTurnEncoder.getPosition() + (setPoint * this.dblTurnMotorConversionFactor), ControlType.kPosition);
        }
    }

    public void setDriveMotorSpeed(double speed) {
        this.driveMotor.set(speed);
    }
}
