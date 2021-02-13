package com.team2137.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.team2137.frc2021.Constants;
import com.team2137.frc2021.util.Motor;
import com.team2137.frc2021.util.Motor.MotorTypes;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private TalonFX flywheelMotor1;
    private TalonFX flywheelMotor2;

    private CANSparkMax preRollerMotor;
    private CANSparkMax hoodMotor;
    
    private PIDController flywheelPIDController = new PIDController(0.15, 0.06, 0.0005); //0.1 0 0 ~
    private CANPIDController hoodPIDController;

    private CANEncoder hoodMotorEncoder;

    private SimpleMotorFeedforward flywheelFeedForward = new SimpleMotorFeedforward(0.4, 0.106, 0.0225);

    private SupplyCurrentLimitConfiguration flywheelCurrentLimit;

    private Constants.StepState mStateHoodHoming = Constants.StepState.STATE_FINISH;

    private int debug = 10; // 1 - 10

    private double dblFlywheelVelocityGoal = 1757.8125;
    private double dblHoodMotorHomingCurrentLimit = 20;

    public Shooter() {
        //Parm 0 - Ramp Speed ~ Parm 1 - P ~ Parm 2 - I ~ Parm 3 - D ~ Parm 4 - Max Vel
        Motor tmpFlyWheelMotor1 = new Motor("flyWheelMotor1", 42, MotorTypes.FALCON, false, 60, 1, "1.5", ".1", "0", "0", "6000"); //Parm 0 - Ramp Speed ~ Parm 1 - P ~ Parm 2 - I ~ Parm 3 - D ~ Parm 4 - Max Vel
        Motor tmpFlyWheelMotor2 = new Motor("flyWheelMotor2", 43, MotorTypes.FALCON, true, 60, 1, "1.5");
        Motor tmpPreRollerMotor = new Motor("preRollerMotor", 44, MotorTypes.NEO550, true, 60, 1, "0");
        //TODO fix gear ratio for hood motor (Rotation Per Degree)
        Motor tmpHoodMotor      = new Motor("hoodMotor", 41, MotorTypes.NEO550, false, 60, 1, "0", "1", "0", "0", "45", "0", "30"); //Parm 0 - Ramp Speed ~ Parm 1 - P ~ Parm 2 - I ~ Parm 3 - D ~ Parm 4 - Max Position deg ~ Parm 5 Min Position deg ~ Parm 6 Zero Current

        this.dblHoodMotorHomingCurrentLimit = Double.parseDouble(tmpHoodMotor.getParm(6));

        this.flywheelMotor1     = new TalonFX(tmpFlyWheelMotor1.getMotorID());
        this.flywheelMotor2     = new TalonFX(tmpFlyWheelMotor2.getMotorID());
        this.preRollerMotor     = new CANSparkMax(tmpPreRollerMotor.getMotorID(), tmpPreRollerMotor.getMotorType().getREVType());
        this.hoodMotor          = new CANSparkMax(tmpHoodMotor.getMotorID(), tmpHoodMotor.getMotorType().getREVType());

        this.hoodMotorEncoder   = this.hoodMotor.getEncoder();
        this.hoodMotorEncoder.setPositionConversionFactor(tmpHoodMotor.getGearRatio());

        this.hoodPIDController  = this.hoodMotor.getPIDController();
        this.hoodPIDController.setP(Double.parseDouble(tmpHoodMotor.getParm(1)));
        this.hoodPIDController.setI(Double.parseDouble(tmpHoodMotor.getParm(2)));
        this.hoodPIDController.setD(Double.parseDouble(tmpHoodMotor.getParm(3)));

        this.flywheelCurrentLimit = new SupplyCurrentLimitConfiguration(true, tmpFlyWheelMotor1.getCurrentLimit(), 80, 2);

        this.flywheelMotor1.configFactoryDefault();
        this.flywheelMotor1.configOpenloopRamp(Double.parseDouble(tmpFlyWheelMotor1.getParm(0)));
        this.flywheelMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.flywheelMotor1.configSupplyCurrentLimit(this.flywheelCurrentLimit);
        this.flywheelMotor1.setInverted(tmpFlyWheelMotor1.inverted());

        this.flywheelMotor2.configFactoryDefault();
        this.flywheelMotor2.configOpenloopRamp(Double.parseDouble(tmpFlyWheelMotor2.getParm(0)));
        this.flywheelMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.flywheelMotor2.configSupplyCurrentLimit(this.flywheelCurrentLimit);
        this.flywheelMotor2.setInverted(tmpFlyWheelMotor2.inverted());
        
        this.preRollerMotor.setSmartCurrentLimit(tmpPreRollerMotor.getCurrentLimit());
        this.preRollerMotor.setOpenLoopRampRate(Double.parseDouble(tmpPreRollerMotor.getParm(0)));
        this.preRollerMotor.setInverted(tmpPreRollerMotor.inverted());

        this.hoodMotor.setSmartCurrentLimit(tmpHoodMotor.getCurrentLimit());
        this.hoodMotor.setInverted(tmpHoodMotor.inverted());
    }//Base is 31 by 21.5

    public void loop() {
        switch (this.mStateHoodHoming) {
            case STATE_INIT:
                this.hoodMotor.set(-.5);
                break;
            case STATE_RUNNING:
                if (this.hoodMotor.getOutputCurrent() > this.dblHoodMotorHomingCurrentLimit) {
                    this.hoodMotor.set(0);
                    this.hoodMotorEncoder.setPosition(0);
                    this.mStateHoodHoming = Constants.StepState.STATE_FINISH;
                }
                break;
            case STATE_FINISH:
                break;
        }

        SmartDashboard.putNumber("PreRoller Power", preRollerMotor.get());
        SmartDashboard.putNumber("Hood Angle", getHoodAngle());

        double flywheelPower = (flywheelFeedForward.calculate(dblFlywheelVelocityGoal / 60) + flywheelPIDController.calculate(getFlywheelVelocity() / 60, dblFlywheelVelocityGoal / 60)) / 12;
        SmartDashboard.putNumber("FlyWheel Power", flywheelPower);
        SmartDashboard.putNumber("FlyWheel Velocity", getFlywheelVelocity());
        this.flywheelMotor1.set(ControlMode.PercentOutput, flywheelPower);
        this.flywheelMotor2.set(ControlMode.PercentOutput, flywheelPower);
    }

    public double getFlywheelVelocity() {
        return (this.flywheelMotor1.getSensorCollection().getIntegratedSensorVelocity() / 2048) * 600;
    }
    public void setFlywheelVelocity(double velocity) {
        dblFlywheelVelocityGoal = velocity;
    }

    public void setPreRollerPower(double speed) {
        this.preRollerMotor.set(speed);
    }

    public double getHoodAngle() {
        return this.hoodMotorEncoder.getPosition();
    }

    public void setHoodAngle(double angle) {
        this.hoodPIDController.setReference(angle, ControlType.kPosition);
    }
    public void zeroHoodAngle() {
        this.mStateHoodHoming = Constants.StepState.STATE_INIT;
    }
}