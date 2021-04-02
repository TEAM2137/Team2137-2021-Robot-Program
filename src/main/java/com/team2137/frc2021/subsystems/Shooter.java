package com.team2137.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.team2137.frc2021.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import java.sql.Driver;
import java.util.TreeMap;

import static com.team2137.frc2021.Constants.*;
import static com.team2137.frc2021.Constants.Shooter.*;

@SuppressWarnings("all")
public class Shooter extends SubsystemBase {

    //Create the motors that are attach to the flywheel on the shooter
    private TalonFX flywheelMotor1;
    private TalonFX flywheelMotor2;

    //Create the PreRoller motor and hood motor for the shooter
    private CANSparkMax preRollerMotor;
    private CANSparkMax hoodMotor;

    //Create the two PID Controller needed for the shooter.
    private PIDController flywheelPIDController; //Flywheel shooter to create fast recovery from shooting a ball
    private CANPIDController hoodPIDController; //PID to position the hood using the integrated PID controller from REV

    //To store the encoder that is built into the motor
    private CANEncoder hoodMotorEncoder;

    //Feed Forward for the shooter to add with the PID Loops
    private SimpleMotorFeedforward flywheelFeedForward;
    private SupplyCurrentLimitConfiguration flywheelCurrentLimit; //Current limited to prevent the motors from burning if jammed

    //Store the state of the Hood. (Weather Homing or moving to position)
    private StepState mStateHoodHoming = StepState.STATE_INIT;
    private double startTime = 0;

    private double dblFlywheelVelocityGoal = 0; //Velocity Goal in RPM for the shooter
    private double dblHoodMotorHomingCurrentLimit = HoodMotorHomingCurrentSignal; //The Current that is needed for the hood to be considered homed
    private double dblHoodMotorTargetAngle = 0; //The goal for the hood position and needed if homing is called during a movement

    private Timer hoodHomingTimer;

    private TreeMap<Double, Translation2d> shooterSampleValues = new TreeMap<>();

    private boolean boolFlywheelIdled = false;

    public Shooter() {
        //Make sure to add current limiting for the hood motor so that it does not burn
//        this.dblHoodMotorHomingCurrentLimit = Double.parseDouble(HoodMotorObject.getParm(6));
        this.dblHoodMotorHomingCurrentLimit = 25; //temp

        shooterSampleValues.put(5.0, new Translation2d(4000.0, 3.0));
        shooterSampleValues.put(10.0 , new Translation2d(4700.0, 17.0));
        shooterSampleValues.put(15.0 , new Translation2d(5300.0, 25.0));
        shooterSampleValues.put(20.0 , new Translation2d(5700.0, 30));

        //Create the motor objects and store them to the respective variables
        this.flywheelMotor1     = new TalonFX(FlyWheelMotorObject1.getMotorID());
        this.flywheelMotor2     = new TalonFX(FlyWheelMotorObject2.getMotorID());
        this.preRollerMotor     = new CANSparkMax(PreRollerMotorObject.getMotorID(), PreRollerMotorObject.getMotorType().getREVType());
        this.hoodMotor          = new CANSparkMax(HoodMotorObject.getMotorID(), HoodMotorObject.getMotorType().getREVType());

        this.preRollerMotor.restoreFactoryDefaults();
        this.hoodMotor.restoreFactoryDefaults();

        // Get and store the encoder and add teh Conversion Factor so the units while setting position is in degrees
        this.hoodMotorEncoder   = this.hoodMotor.getEncoder();
        this.hoodMotorEncoder.setPositionConversionFactor(HoodMotorObject.getGearRatio());

        // Set up the hood PID Controller
        this.hoodPIDController  = this.hoodMotor.getPIDController();
        this.hoodPIDController.setP(HoodMotorObject.getPID().getP());
        this.hoodPIDController.setI(HoodMotorObject.getPID().getI());
        this.hoodPIDController.setD(HoodMotorObject.getPID().getD());
        this.hoodPIDController.setFF(0.001);

        this.flywheelCurrentLimit = new SupplyCurrentLimitConfiguration(true, FlyWheelMotorObject1.getCurrentLimit(), 80, 2);

        //Configure the Flywheel motor with al of the values
        this.flywheelMotor1.configFactoryDefault();
        this.flywheelMotor1.configOpenloopRamp(FlyWheelMotorObject1.getRampRate()); //Using Ramp to stop the belt from skipping and it is in Second to full speed
        this.flywheelMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.flywheelMotor1.configSupplyCurrentLimit(this.flywheelCurrentLimit);
        this.flywheelMotor1.setInverted(FlyWheelMotorObject1.inverted());
        this.flywheelMotor1.setNeutralMode(NeutralMode.Coast);

        this.flywheelMotor2.configFactoryDefault();
        this.flywheelMotor2.configOpenloopRamp(FlyWheelMotorObject1.getRampRate());
        this.flywheelMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.flywheelMotor2.configSupplyCurrentLimit(this.flywheelCurrentLimit);
        this.flywheelMotor2.setInverted(FlyWheelMotorObject2.inverted());
        this.flywheelMotor2.setNeutralMode(NeutralMode.Coast);

        this.flywheelPIDController = FlyWheelMotorObject1.getPID().getWPIPIDController();
        this.flywheelFeedForward = FlyWheelMotorObject1.getMotorFeedForwardController();

        this.preRollerMotor.setSmartCurrentLimit(PreRollerMotorObject.getCurrentLimit());
        this.preRollerMotor.setOpenLoopRampRate(PreRollerMotorObject.getRampRate());
        this.preRollerMotor.setInverted(PreRollerMotorObject.inverted());

        this.hoodMotor.setSmartCurrentLimit(HoodMotorObject.getCurrentLimit());
        this.hoodMotor.setInverted(HoodMotorObject.inverted());

        this.mStateHoodHoming = StepState.STATE_INIT;

        this.hoodHomingTimer = new Timer();
    }

    @Override
    public void periodic() {
        /*  For every loop go through a state machine
            All other states than FINISH and NOT_STARTED are for homing the hood
         */
        switch (this.mStateHoodHoming) {
            case STATE_INIT: //Start the homing process of the hood
                if (DriverStation.getInstance().isEnabled()) {
                    this.hoodHomingTimer.reset();
                    this.hoodHomingTimer.start();
                    this.hoodMotor.set(-0.25); //Reverse the hood back to zero direction
                    this.mStateHoodHoming = StepState.STATE_RUNNING; //Move to the looping stage
                }
                break;
            case STATE_RUNNING: //Loop and check if the hood is a home yet
                //If the motor is drawing to much power stop the power and set that as the home position and continue commands before it
                if (this.hoodMotor.getOutputCurrent() > this.dblHoodMotorHomingCurrentLimit && System.currentTimeMillis() - startTime > 200) {
                    this.hoodMotorEncoder.setPosition(-8.35917);
                    this.hoodMotor.set(0);
                    this.hoodPIDController.setReference(dblHoodMotorTargetAngle, ControlType.kPosition);
                    this.mStateHoodHoming = Constants.StepState.STATE_FINISH;
                }
                break;
            case STATE_FINISH: //The Homing process is finished
                break;
        }

        double flywheelPower = MathUtil.clamp((flywheelFeedForward.calculate(dblFlywheelVelocityGoal / 60) + flywheelPIDController.calculate(getFlywheelVelocity() / 60, dblFlywheelVelocityGoal / 60)) / 12, 0, 1);

        if (boolFlywheelIdled)
            flywheelPower *= FlywheelIdlePercent;

        if (flywheelPower < 0)
            flywheelPower = 0;

        SmartDashboard.putNumber("FlyWheel Velocity", getFlywheelVelocity());
        SmartDashboard.putNumber("Flywheel Goal", dblFlywheelVelocityGoal);
        this.flywheelMotor1.set(ControlMode.PercentOutput, flywheelPower);
        this.flywheelMotor2.set(ControlMode.PercentOutput, flywheelPower);
    }

    /**
     * Using integrated sensor get the velocity of the the motor
     * @return Flywheel Speed in RPM
     */
    public double getFlywheelVelocity() {
        //Get the Integrated Sensor velocity divide by 2048 counts per rotation to get rotations per 100 milliseconds and times by 600 to get RPM
        return (this.flywheelMotor1.getSelectedSensorVelocity() / 2048) * 600;
    }

    public boolean isFlywheelAtTarget(double width) {
        return (Math.abs(getFlywheelVelocity() - dblFlywheelVelocityGoal) < width) && !isIdle();
    }

    public void idleFlyWheel() {
        preRollerMotor.set(0);
        boolFlywheelIdled = true;
    }

    public void revFlywheel() {
        preRollerMotor.set(1);
        boolFlywheelIdled = false;
    }

    /**
     * Sets the velocity goal of the flywheel for the PID Controller
     * @param velocity Velocity in RMP
     */
    public void setFlywheelVelocity(double velocity) {
        dblFlywheelVelocityGoal = velocity;
    }

    /**
     * Sets raw power to the PreRoller on the shooter
     * @param speed 0~1 value for PreRoller Speed
     */
    public void setPreRollerPower(double speed) {
        this.preRollerMotor.set(speed);
    }

    /**
     * Sets raw power to the PreRoller on the shooter using {@link #setPreRollerPower(double)} to default power of 1
     */
    public void enablePreRoller() {
        setPreRollerPower(1);
    }

    /**
     * Sets raw power to the PreRoller on the shooter using {@link #setPreRollerPower(double)} to default power of 0
     */
    public void disablePreRoller() {
        setPreRollerPower(0);
    }


    /**
     * Using the built in Hood motor encoder returns the position of the hood
     * @return Position of the hood in degrees
     */
    public double getHoodAngle() {
        return this.hoodMotorEncoder.getPosition();
    }

    /**
     * Sets the angle of the hood on the shooter and is overrided by a homing operation if in progress and will resume after homing is done
     * @param angle Target angle of the shooter hood (>0)
     */
    public void setHoodAngle(double angle) {
        this.dblHoodMotorTargetAngle = angle; //Store the value in the variable so that once homing is done it will remember the position
        if (this.mStateHoodHoming == StepState.STATE_FINISH || this.mStateHoodHoming == StepState.STATE_NOT_STARTED)
            this.hoodPIDController.setReference(angle, ControlType.kPosition); //If the hood it homing then do not change the speed until it is done
    }

    public void setPreset(ShooterPresets preset) {
        setFlywheelVelocity(preset.flywheelSpeed);
        setHoodAngle(preset.hoodAngle);
        setPreRollerPower(preset.prerollerPower);
    }

    /**
     * Initiates a homing command for the hood on the shooter and override a target position resuming after homing
     */
    public void zeroHoodAngle() {
        this.mStateHoodHoming = Constants.StepState.STATE_INIT;
    }

    /**
     * Returns if the hood has been homed during the match yet
     * @return Hood Homing Status
     */
    public boolean isHoodZeroed() {
        return this.mStateHoodHoming != StepState.STATE_NOT_STARTED;
    }

    private double getHoodAngle(double distanceFeet) {
        var min = shooterSampleValues.floorEntry(distanceFeet);
        var max = shooterSampleValues.ceilingEntry(distanceFeet);
        if(min == null)
            return max.getValue().getY();
        else if(max == null)
            return min.getValue().getY();
        else if (min.getKey() == distanceFeet)
            return min.getValue().getY();
        else if (max.getKey() == distanceFeet)
            return max.getValue().getY();
        else {
            double amp = max.getKey() - min.getKey();
            double high = ((max.getKey() - distanceFeet) / amp) * max.getValue().getY();
            double low = ((distanceFeet - min.getKey()) / amp) * min.getValue().getY();
            return high + low;
        }
    }

    private double getFlywheelSpeed(double distanceFeet) {
        var min = shooterSampleValues.floorEntry(distanceFeet);
        var max = shooterSampleValues.ceilingEntry(distanceFeet);
        if(min == null)
            return max.getValue().getX();
        else if(max == null)
            return min.getValue().getX();
        else if (min.getKey() == distanceFeet)
            return min.getValue().getX();
        else if (max.getKey() == distanceFeet)
            return max.getValue().getX();
        else {
            double amp = max.getKey() - min.getKey();
            double high = ((max.getKey() - distanceFeet) / amp) * max.getValue().getX();
            double low = ((distanceFeet - min.getKey()) / amp) * min.getValue().getX();
            return high + low;
        }
    }

    public void setShooterPosisition(double distanceFeet) {
        setFlywheelVelocity(getFlywheelSpeed(distanceFeet));
        setHoodAngle(getHoodAngle(distanceFeet));
        revFlywheel();
    }

    public boolean isIdle() {
        return boolFlywheelIdled;
    }
}