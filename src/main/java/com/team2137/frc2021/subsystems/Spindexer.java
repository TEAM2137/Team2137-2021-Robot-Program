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

    private DoubleSolenoid ballStop;

    /**
     * Creates a Spindexer from the constants
     */
    public Spindexer() {
        this.motor = new TalonSRX(Constants.Spindexer.motorID);
        this.motor.setInverted(Constants.Spindexer.invertMotor);
        this.motor.configContinuousCurrentLimit(Constants.Spindexer.currentLimit);
        this.motor.setNeutralMode(NeutralMode.Brake);
        this.motor.configClosedloopRamp(Constants.Spindexer.voltageRamp);
        this.motor.configOpenloopRamp(Constants.Spindexer.voltageRamp);

        this.ballStop = new DoubleSolenoid(Constants.Spindexer.solenoidForwardID, Constants.Spindexer.solenoidReverseID);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindexer Power", motor.getMotorOutputPercent());
        SmartDashboard.putBoolean("Spindexer Ball Stop On", isBallStopEnabled());
    }

    /**
     * @param power power from -1 to 1 to drive the spindexer
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public void setBallStop(BallStopState state) {
        ballStop.set(state.value);
    }

    public boolean isBallStopEnabled() {
        return ballStop.get() == BallStopState.Enabled.value;
    }

    public enum BallStopState {
        Enabled(DoubleSolenoid.Value.kForward),
        Disabled(DoubleSolenoid.Value.kReverse),
        ;

        DoubleSolenoid.Value value;

        BallStopState(DoubleSolenoid.Value value) {
            this.value = value;
        }
    }
}
