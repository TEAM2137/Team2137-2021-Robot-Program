package com.team2137.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2137.frc2021.Constants;
import com.team2137.frc2021.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.CANifier;

public class Spindexer extends SubsystemBase {

    private TalonSRX motor;

    private CANifier canifier;
    private DoubleSolenoid ballStopper;

    /**
     * Creates a Spindexer from the constants
     */
    public Spindexer(CANifier canifier) {
        this.motor = new TalonSRX(Constants.Spindexer.motorID);
        this.motor.setInverted(Constants.Spindexer.invertMotor);
        this.motor.configContinuousCurrentLimit(Constants.Spindexer.currentLimit);
        this.motor.setNeutralMode(NeutralMode.Brake);
        this.motor.configClosedloopRamp(Constants.Spindexer.voltageRamp);
        this.motor.configOpenloopRamp(Constants.Spindexer.voltageRamp);

        this.ballStopper = new DoubleSolenoid(Constants.Spindexer.solenoidForwardID, Constants.Spindexer.solenoidReverseID);

        this.canifier = canifier;

        Robot.addOnEnabled(this::enableSpindexer);
        Robot.addOnDisabled(this::disabledSpindexer);
    }

    @Override
    public void periodic() {
    }

    /**
     * @param power power from -1 to 1 to drive the spindexer
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public void enableSpindexer() {
        SmartDashboard.putBoolean("Spindexer Enabled", true);
        setPower(1);
    }

    public boolean isBallStopperEnabled() {
        return ballStopper.get() == DoubleSolenoid.Value.kForward;
    }

    public void disabledSpindexer() {
        SmartDashboard.putBoolean("Spindexer Enabled", false);
        setPower(0);
    }

    public void enableBallStopper() {
        SmartDashboard.putBoolean("BallStopper Enabled", true);
        ballStopper.set(DoubleSolenoid.Value.kForward);
    }

    public void disabledBallStopper() {
        SmartDashboard.putBoolean("BallStopper Enabled", false);
        ballStopper.set(DoubleSolenoid.Value.kOff);
    }
}
