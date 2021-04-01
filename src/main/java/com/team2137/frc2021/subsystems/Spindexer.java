package com.team2137.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.team2137.frc2021.Constants;
import com.team2137.frc2021.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.CANifier;

import java.sql.Driver;

public class Spindexer extends SubsystemBase {

    private TalonSRX motor;

    private DoubleSolenoid ballStop;

    /**
     * Creates a Spindexer from the constants
     */
    public Spindexer() {
        this.motor = new TalonSRX(Constants.Spindexer.motorID);
        this.motor.configFactoryDefault();
        this.motor.setInverted(Constants.Spindexer.invertMotor);
        this.motor.setNeutralMode(NeutralMode.Brake);

        this.ballStop = new DoubleSolenoid(Constants.Spindexer.solenoidForwardID, Constants.Spindexer.solenoidReverseID);
    }

    @Override
    public void periodic() {
//        SmartDashboard.putNumber("Spindexer Power", motor.getMotorOutputPercent());
//        SmartDashboard.putBoolean("Spindexer Ball Stop On", isBallStopEnabled());
    }

    /**
     * @param power power from -1 to 1 to drive the spindexer
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public void setBallStop(BallStopState state) {
//        ballStop.set(state.value);
    }

    public boolean isBallStopEnabled() {
        return ballStop.get() == BallStopState.Enabled.value;
    }

    public enum BallStopState {
        Enabled(DoubleSolenoid.Value.kReverse),
        Disabled(DoubleSolenoid.Value.kForward),
        ;

        DoubleSolenoid.Value value;

        BallStopState(DoubleSolenoid.Value value) {
            this.value = value;
        }
    }
}
