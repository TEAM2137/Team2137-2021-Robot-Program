package com.team2137.frc2021.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2137.frc2021.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    //CANSparkMax motor;
    TalonSRX motor;
    
    DoubleSolenoid cylinder;

    private IntakeState state;

    public Intake() {
        this.motor = new TalonSRX(Constants.Intake.motorID);
        //CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);
        this.cylinder = new DoubleSolenoid(Constants.Intake.cylinderForwardID, Constants.Intake.cylinderReverseID);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeSpeed", motor.getActiveTrajectoryVelocity());
        SmartDashboard.putString("Cylinder State", cylinder.get().toString());
    }

    public void setCylinderState(DoubleSolenoid.Value state) {
        cylinder.set(state);
    }

    public void setMotorPowerRaw(double power) {
        motor.set(ControlMode.PercentOutput, power);;
    }

    public void setIntakeState(IntakeState state) {
        setMotorPowerRaw(state.motorPower);
        setCylinderState(state.cylinderState);
        this.state = state;
    }

    public IntakeState getIntakeState() {
        return state;
    }

    public enum IntakeState {
        Retracted(0, DoubleSolenoid.Value.kReverse),
        Running(1, DoubleSolenoid.Value.kForward),
        ;

        public final double motorPower;
        public final DoubleSolenoid.Value cylinderState;
        IntakeState(double motorPower, DoubleSolenoid.Value cylinderState) {
            this.motorPower = motorPower;
            this.cylinderState = cylinderState;
        }
    }
}