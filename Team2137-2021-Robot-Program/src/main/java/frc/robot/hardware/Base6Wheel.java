package frc.robot.hardware;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.base.Feedforward;
import frc.robot.base.HardwareDriveTrain;
import frc.robot.Constants;

public class Base6Wheel extends HardwareDriveTrain {

    private CANSparkMax leftMotor1;
    private CANSparkMax leftMotor2;    
    private CANSparkMax rightMotor1;
    private CANSparkMax rightMotor2;

    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private CANPIDController leftPIDController;
    private CANPIDController rightPIDController;
    
    private Feedforward feedforward;

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

        this.leftPIDController = this.leftMotor1.getPIDController();
        this.rightPIDController = this.rightMotor1.getPIDController();
        
        this.feedforward = new Feedforward(0.0, 0.0, 0.0);
        
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
        boolean shiftTo = (state == ShifterStates.High);
        this.baseShifter.set(Constants.boolShifterInvert ? !shiftTo : shiftTo);
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

    public void setPIDVelocityOutput(double leftVelocity, double rightVelocity) {
        this.leftPIDController.setFF(this.feedforward.calculate(leftVelocity));
        this.rightPIDController.setFF(this.feedforward.calculate(rightVelocity));

        this.leftPIDController.setReference(leftVelocity, ControlType.kVelocity);
        this.rightPIDController.setReference(rightVelocity, ControlType.kVelocity);
    }
    public void setPIDValues(ShifterStates state) {
        int PIDState = (state == ShifterStates.High) ? 1 : 0;

        this.leftPIDController.setP(Constants.dblBaseP[PIDState]);
        this.leftPIDController.setI(Constants.dblBaseI[PIDState]);
        this.leftPIDController.setD(Constants.dblBaseD[PIDState]);

        this.rightPIDController.setP(Constants.dblBaseP[PIDState]);
        this.rightPIDController.setI(Constants.dblBaseI[PIDState]);
        this.rightPIDController.setD(Constants.dblBaseD[PIDState]);

        this.feedforward.setSVA(Constants.dblBaseS[PIDState], Constants.dblBaseV[PIDState], Constants.dblBaseA[PIDState]);
    }
}