package frc.robot.base;

import com.revrobotics.ControlType;

public interface HardwareDriveTrain extends HardwareBasicDriveTrain {
    enum ShifterStates {
        High,
        Low
    }

    boolean boolPIDEnabled = false;

    default void setLeft1PID(ControlType control, double value) {
        setLeft1Power(value);
    }
    default void setLeft2PID(ControlType control, double value) {
        setLeft2Power(value);
    }
    default void setRight1PID(ControlType control, double value) {
        setRight1Power(value);
    }
    default void setRight2PID(ControlType control, double value) {
        setRight2Power(value);
    }

    default void setLeftPID(ControlType control, double value) {
        setLeft1PID(control, value);
        setLeft2PID(control, value);
    }
    default void setRightPID(ControlType control, double value) {
        setRight1PID(control, value);
        setRight2PID(control, value);
    }

    void setLeft1Power(double leftM1);
    void setLeft2Power(double leftM2);
    void setRight1Power(double rightM1);
    void setRight2Power(double rightM2);

    default void setLeftPower(double power) {
        setLeft1Power(power);
        setLeft2Power(power);
    }
    default void setRightPower(double power) {
        setRight1Power(power);
        setRight2Power(power);
    }
    default void setAllPower(double speed) {
        setLeft1Power(speed);
        setLeft2Power(speed);
        setRight1Power(speed);
        setRight2Power(speed);
    }

    double getLeftMotor1Position();
    double getLeftMotor2Position();
    double getRightMotor1Position();
    double getRightMotor2Position();

    default double getLeftMotorPosition() {
        return (getLeftMotor1Position() + getLeftMotor2Position()) / 2;
    }
    default double getRightMotorPosition() {
        return (getRightMotor1Position() + getRightMotor2Position()) / 2;
    }

    void setLeftMotor1TargetPosition(double target);
    void setLeftMotor2TargetPosition(double target);
    void setRightMotor1TargetPosition(double target);
    void setRightMotor2TargetPosition(double target);

    double getLeftMotor1TargetPosition();
    double getLeftMotor2TargetPosition();
    double getRightMotor1TargetPosition();
    double getRightMotor2TargetPosition();

    default void setLeftMotorTargetPosition(double target) {
        setLeftMotor1TargetPosition(target);
        setLeftMotor2TargetPosition(target);
    }
    default void setRightMotorTargetPosition(double target) {
        setRightMotor1TargetPosition(target);
        setRightMotor2TargetPosition(target);
    }

    default double getLeftMotorTargetPosition() {
        return (getLeftMotor1TargetPosition() + getLeftMotor2TargetPosition()) / 2;
    }
    default double getRightMotorTargetPosition() {
        return (getRightMotor1TargetPosition() + getRightMotor2TargetPosition()) / 2;
    }

    default void addLeftMotor1TargetPosition(double target) {
        setLeftMotor1TargetPosition(getLeftMotor1Position() + target);
    }
    default void addLeftMotor2TargetPosition(double target) {
        setLeftMotor2TargetPosition(getLeftMotor2Position() + target);
    }
    default void addRightMotor1TargetPosition(double target) {
        setRightMotor1TargetPosition(getRightMotor1Position() + target);
    }
    default void addRightMotor2TargetPosition(double target) {
        setRightMotor2TargetPosition(getRightMotor2Position() + target);
    }

    default void addLeftMotorTargetPosition(double target) {
        addLeftMotor1TargetPosition(target);
        addLeftMotor2TargetPosition(target);
    }
    default void addRightMotorTargetPosition(double target) {
        addRightMotor1TargetPosition(target);
        addRightMotor2TargetPosition(target);
    }
}