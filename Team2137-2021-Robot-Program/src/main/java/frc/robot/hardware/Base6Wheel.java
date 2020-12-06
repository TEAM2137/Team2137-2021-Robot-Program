package frc.robot.hardware;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.base.Feedforward;
import frc.robot.base.HardwareDriveTrain;
import frc.robot.util.FileLogger;
import frc.robot.util.Motor;
import frc.robot.util.XMLSettingReader;
import frc.robot.Constants;

public class Base6Wheel implements HardwareDriveTrain {

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

    private double dblLeftMotor1TargetPosition = 0;
    private double dblLeftMotor2TargetPosition = 0;
    private double dblRightMotor1TargetPosition = 0;
    private double dblRightMotor2TargetPosition = 0;

    @Override
	public void init(XMLSettingReader reader, FileLogger log) {
        Motor tmpLM1 = reader.getMotor("LM1");
        Motor tmpLM2 = reader.getMotor("LM2");
        Motor tmpRM1 = reader.getMotor("RM1");
        Motor tmpRM2 = reader.getMotor("RM2");

        leftMotor1 = new CANSparkMax(tmpLM1.getMotorID(), MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(tmpLM2.getMotorID(), MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(tmpRM1.getMotorID(), MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(tmpRM2.getMotorID(), MotorType.kBrushless);
        
        this.leftMotor2.follow(this.leftMotor1);
        this.rightMotor2.follow(this.rightMotor1);

        this.leftMotor1.setInverted(tmpLM1.inverted());
        this.leftMotor2.setInverted(tmpLM2.inverted());
        this.rightMotor1.setInverted(tmpRM1.inverted());
        this.rightMotor2.setInverted(tmpRM2.inverted());

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
        this.baseShifter.set(Constants.boolShifterInvert != shiftTo);
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

	public double getLeftMotorPosition() {
		return this.leftEncoder.getPosition();
	}
	public double getRightMotorPosition() {
        return this.rightEncoder.getPosition();
	}

    @Override
    public void setLeftMotor1TargetPosition(double target) {
        dblLeftMotor1TargetPosition = target;
    }
    @Override
    public void setLeftMotor2TargetPosition(double target) {
        dblLeftMotor2TargetPosition = target;
    }
    @Override
    public void setRightMotor1TargetPosition(double target) {
        dblRightMotor1TargetPosition = target;
    }
    @Override
    public void setRightMotor2TargetPosition(double target) {
        dblRightMotor2TargetPosition = target;
    }

    @Override
    public double getLeftMotor1TargetPosition() {
        return dblLeftMotor1TargetPosition;
    }
    @Override
    public double getLeftMotor2TargetPosition() {
        return dblLeftMotor2TargetPosition;
    }
    @Override
    public double getRightMotor1TargetPosition() {
        return dblRightMotor1TargetPosition;
    }
    @Override
    public double getRightMotor2TargetPosition() {
        return dblRightMotor2TargetPosition;
    }

    public void setLeft1Power(double leftM1) {
		leftMotor1.set(Constants.clipBaseSpeed(leftM1));
	}
	public void setLeft2Power(double leftM2) {
		leftMotor2.set(Constants.clipBaseSpeed(leftM2));
	}
	public void setRight1Power(double rightM1) {
		rightMotor1.set(Constants.clipBaseSpeed(rightM1));
	}
	public void setRight2Power(double rightM2) {
		rightMotor2.set(Constants.clipBaseSpeed(rightM2));
	}
}