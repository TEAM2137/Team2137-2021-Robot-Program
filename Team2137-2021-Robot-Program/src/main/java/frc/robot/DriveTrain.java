package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.base.HardwareDriveTrain;
import frc.robot.Constants;

public class DriveTrain extends HardwareDriveTrain {

    private CANSparkMax leftMotor1;
    private CANSparkMax leftMotor2;    
    private CANSparkMax rightMotor1;
    private CANSparkMax rightMotor2;

    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private CANPIDController leftPIDController;
    private CANPIDController rightPIDController;
    
    private SimpleMotorFeedforward simpleMotorFeedforward;

    private double P, I, D;
    private double S, V, A;

    private Solenoid baseShifter;
    private ShifterStates shifterState;

    @Override
	public void init() {
        leftMotor1 = new CANSparkMax(Constants.intLM1ID, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(Constants.intLM2ID, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(Constants.intRM1ID, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(Constants.intRM2ID, MotorType.kBrushless);

        this.leftMotor2.follow(this.leftMotor1);
        this.rightMotor2.follow(this.rightMotor1);

        this.leftMotor1.setInverted(Constants.boolLM1Invert);
        this.leftMotor2.setInverted(Constants.boolLM2Invert);
        this.rightMotor1.setInverted(Constants.boolRM1Invert);
        this.rightMotor2.setInverted(Constants.boolRM2Invert);

        this.leftEncoder = new CANEncoder(this.leftMotor1, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);
        this.rightEncoder = new CANEncoder(this.rightMotor1, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);

        this.P = Constants.dblBaseP;
        this.I = Constants.dblBaseI;
        this.D = Constants.dblBaseD;
     
        this.S = Constants.dblBaseS;
        this.V = Constants.dblBaseV;
        this.A = Constants.dblBaseA;

        this.leftPIDController = new CANPIDController(this.leftMotor1);
        this.rightPIDController = new CANPIDController(this.rightMotor1);
        
        this.simpleMotorFeedforward = new SimpleMotorFeedforward(this.S, this.V, this.A);

        this.leftPIDController.setP(this.P);
        this.leftPIDController.setI(this.I);
        this.leftPIDController.setD(this.D);

        this.rightPIDController.setP(this.P);
        this.rightPIDController.setI(this.I);
        this.rightPIDController.setD(this.D);
        
        this.leftPIDController.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        this.rightPIDController.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        
        this.leftPIDController.setFeedbackDevice(this.leftEncoder);
        this.rightPIDController.setFeedbackDevice(this.rightEncoder);

        this.shifterState = ShifterStates.Low;
        this.baseShifter = new Solenoid(Constants.intSolenoidID);
    }

    public boolean smartVoltageGearShift() {
        double averageVoltage = (this.leftMotor1.getBusVoltage() + this.rightMotor1.getBusVoltage()) / 2;

        if (averageVoltage >= Constants.dblHighGearVoltageLimit && this.shifterState != ShifterStates.Low) {
            shiftGear(ShifterStates.Low);
            return true;
        }

        return false;
    }
    public void smartVelocityGearShift(){
        if(smartVoltageGearShift()) return;
        double averageVelocity = (this.leftEncoder.getVelocity() + this.rightEncoder.getVelocity()) / 2;

        if (averageVelocity >= Constants.dblLowGearMaxVelocity && this.shifterState != ShifterStates.High) {
            shiftGear(ShifterStates.High);
        } else if (averageVelocity <= Constants.dblHighHearMinVelocity && this.shifterState != ShifterStates.Low) {
            shiftGear(ShifterStates.Low);
        }
    }

    public void shiftGear(ShifterStates state){
        this.baseShifter.se
    }

    public void setPIDVelocityOutput(double leftVelocity, double rightVelocity) {
        this.leftPIDController.setFF(this.simpleMotorFeedforward.calculate(leftVelocity));
        this.rightPIDController.setFF(this.simpleMotorFeedforward.calculate(rightVelocity));

        this.leftPIDController.setReference(leftVelocity, ControlType.kVelocity);
        this.rightPIDController.setReference(rightVelocity, ControlType.kVelocity);
    }

    public void setAllPower(double power){
        setLeftPower(power);
        setRightPower(power);
    }
    public void setLeftPower(double power){
        leftMotor1.set(Constants.clipBaseSpeed(power));
        leftMotor2.set(Constants.clipBaseSpeed(power));
    }
    public void setRightPower(double power){
        rightMotor1.set(Constants.clipBaseSpeed(power));
        rightMotor2.set(Constants.clipBaseSpeed(power));
    }
}