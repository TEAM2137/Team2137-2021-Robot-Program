package com.team2137.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2137.frc2021.Constants;
import com.team2137.frc2021.commands.SpindexerControl;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.CANifier;

public class Spindexer extends SubsystemBase {

    private TalonSRX motor;

    private CANifier canifier;

    private SpindexerControl controlCommand;

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

        this.canifier = canifier;

        this.controlCommand = new SpindexerControl(this);
        setDefaultCommand(this.controlCommand);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindexer speed", motor.getMotorOutputPercent());
        SmartDashboard.putBoolean("Photoeye 1", getHopperFirstPhotoeye());
        SmartDashboard.putBoolean("Photoeye 2", getHopperSecondPhotoeye());
        SmartDashboard.putBoolean("Photoeye Fifth ball", getFifthBallPhotoeye());
    }

    /**
     * @param power power from -1 to 1 to drive the spindexer
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    /**
     * @return the state of the first photoeye in the hopper
     */
    public boolean getHopperFirstPhotoeye() {
        return canifier.getGeneralInput(Constants.Spindexer.firstHopperPhotoeyePin);
    }

    /**
     * @return the state of the second photoeye in the hopper
     */
    public boolean getHopperSecondPhotoeye() {
        return canifier.getGeneralInput(Constants.Spindexer.secondHopperPhotoeyePin);
    }

    /**
     * @return the state of the fifth ball detection photoeye in the spindexer
     */
    public boolean getFifthBallPhotoeye() {
        return canifier.getGeneralInput(Constants.Spindexer.fifthBallPhotoeyePin);
    }
}
