package frc.robot.hardware;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.EncoderTypes;
import frc.robot.Constants.StepState;
import frc.robot.base.Feedforward;
import frc.robot.base.HardwareDriveTrain;
import frc.robot.util.Encoder;
import frc.robot.util.FileLogger;
import frc.robot.util.Motor;
import frc.robot.util.XMLSettingReader;

import javax.annotation.Nullable;

public class BaseSwerve implements HardwareDriveTrain {

    private CANSparkMax lm1;
    private CANSparkMax lm2;
    private CANSparkMax rm1;
    private CANSparkMax rm2;

    private CANSparkMax lpm1;
    private CANSparkMax lpm2;
    private CANSparkMax rpm1;
    private CANSparkMax rpm2;

    private CANEncoder leftEncoder1;
    private CANEncoder leftEncoder2;
    private CANEncoder rightEncoder1;
    private CANEncoder rightEncoder2;

    private CANEncoder leftPivotEncoder1;
    private CANEncoder leftPivotEncoder2;
    private CANEncoder rightPivotEncoder1;
    private CANEncoder rightPivotEncoder2;

    private CANCoder leftPivotRotationEncoder1;
    private CANCoder leftPivotRotationEncoder2;
    private CANCoder rightPivotRotationEncoder1;
    private CANCoder rightPivotRotationEncoder2;

    private CANPIDController leftPIDController1;
    private CANPIDController leftPIDController2;
    private CANPIDController rightPIDController1;
    private CANPIDController rightPIDController2;

    private CANPIDController leftPivotPIDController1;
    private CANPIDController leftPivotPIDController2;
    private CANPIDController rightPivotPIDController1;
    private CANPIDController rightPivotPIDController2;

    private double dblPivotCountPerDegree = 0;

    private double dblLeftMotor1TargetPosition = 0;
    private double dblLeftMotor2TargetPosition = 0;
    private double dblRightMotor1TargetPosition = 0;
    private double dblRightMotor2TargetPosition = 0;

    private double dblLeftMotor1ZeroPosition = 0;
    private double dblLeftMotor2ZeroPosition = 0;
    private double dblRightMotor1ZeroPosition = 0;
    private double dblRightMotor2ZeroPosition = 0;

    private Feedforward feedforward;
    
	@Override
	public void init(@Nullable XMLSettingReader reader, @Nullable FileLogger log) {
        Motor tmpLM1, tmpLM2, tmpRM1, tmpRM2, tmpLPM1, tmpLPM2, tmpRPM1, tmpRPM2;
        Encoder tmpLPE1, tmpLPE2, tmpRPE1, tmpRPE2; //Left Pivot Endcoder 1

        if(reader != null) {
            tmpLM1 = reader.getMotor("LM1");
            tmpLM2 = reader.getMotor("LM2");
            tmpRM1 = reader.getMotor("RM1");
            tmpRM2 = reader.getMotor("RM2");

            tmpLPM1 = reader.getMotor("LPM1");
            tmpLPM2 = reader.getMotor("LPM2");
            tmpRPM1 = reader.getMotor("RPM1");
            tmpRPM2 = reader.getMotor("RPM2");
        } else {
            tmpLM1 = new Motor("LM1", 10, Constants.MotorTypes.NEO, false, 80, s);
            tmpLM2 = new Motor("LM2", 15, Constants.MotorTypes.NEO, false, 80, s);
            tmpRM1 = new Motor("RM1", 20, Constants.MotorTypes.NEO, false, 80, s);
            tmpRM2 = new Motor("RM2", 25, Constants.MotorTypes.NEO, false, 80, s);

            tmpLPM1 = new Motor("LPM1", 11, Constants.MotorTypes.NEO, false, 80, 8.53); //TODO fix
            tmpLPM2 = new Motor("LPM1", 16, Constants.MotorTypes.NEO, false, 80, 8.53);
            tmpRPM1 = new Motor("LPM1", 21, Constants.MotorTypes.NEO, false, 80, 8.35);
            tmpRPM2 = new Motor("LPM1", 26, Constants.MotorTypes.NEO, false, 80, 8.53);
            
            tmpLPE1 = new Encoder("LPE1", 12, EncoderTypes.CTRE_MAG_ABS, false);
            tmpLPE2 = new Encoder("LPE2", 17, EncoderTypes.CTRE_MAG_ABS, false);
            tmpRPE1 = new Encoder("RPE1", 22, EncoderTypes.CTRE_MAG_ABS, false);
            tmpRPE2 = new Encoder("RPE2", 27, EncoderTypes.CTRE_MAG_ABS, false);
        }

        this.lm1 = new CANSparkMax(tmpLM1.getMotorID(), MotorType.kBrushless);
        this.lm2 = new CANSparkMax(tmpLM2.getMotorID(), MotorType.kBrushless);
        this.rm1 = new CANSparkMax(tmpRM1.getMotorID(), MotorType.kBrushless);
        this.rm2 = new CANSparkMax(tmpRM2.getMotorID(), MotorType.kBrushless);

        this.lpm1 = new CANSparkMax(tmpLPM1.getMotorID(), MotorType.kBrushless);
        this.lpm2 = new CANSparkMax(tmpLPM2.getMotorID(), MotorType.kBrushless);
        this.rpm1 = new CANSparkMax(tmpRPM1.getMotorID(), MotorType.kBrushless);
        this.rpm2 = new CANSparkMax(tmpRPM2.getMotorID(), MotorType.kBrushless);

        this.lm1.setInverted(tmpLM1.inverted());
        this.lm2.setInverted(tmpLM2.inverted());
        this.rm1.setInverted(tmpRM1.inverted());
        this.rm2.setInverted(tmpRM2.inverted());

        this.lpm1.setInverted(tmpLPM1.inverted());
        this.lpm2.setInverted(tmpLPM2.inverted());
        this.rpm1.setInverted(tmpRPM1.inverted());
        this.rpm2.setInverted(tmpRPM2.inverted());

        this.leftEncoder1 = this.lm1.getEncoder(EncoderType.kQuadrature, 4096);
        this.leftEncoder2 = this.lm2.getEncoder(EncoderType.kQuadrature, 4096);
        this.rightEncoder1 = this.rm1.getEncoder(EncoderType.kQuadrature, 4096);
        this.rightEncoder2 = this.rm2.getEncoder(EncoderType.kQuadrature, 4096);

        this.leftPivotEncoder1 = new CANEncoder(this.lm1, EncoderType.kQuadrature, 4096);
        this.leftPivotEncoder2 = new CANEncoder(this.lm2, EncoderType.kQuadrature, 4096);
        this.rightPivotEncoder1 = new CANEncoder(this.rm1, EncoderType.kQuadrature, 4096);
        this.rightPivotEncoder2 = new CANEncoder(this.rm2, EncoderType.kQuadrature, 4096);

        //this.leftPivotRotationEncoder1 = new CANEncoder(EncoderType.kQuadrature, tmpLPE1.getEncoderID());
        //this.leftPivotRotationEncoder2 = new CANEncoder(tmpLPE2.getEncoderID());
        //this.rightPivotRotationEncoder1 = new CANEncoder(tmpRPE1.getEncoderID());
        //this.rightPivotRotationEncoder2 = new CANEncoder(tmpRPE2.getEncoderID());

        this.leftPIDController1 = this.lm1.getPIDController();
        this.leftPIDController2 = this.lm2.getPIDController();
        this.rightPIDController1 = this.rm1.getPIDController();
        this.rightPIDController2 = this.rm2.getPIDController();
        
        this.leftPivotPIDController1 = this.lpm1.getPIDController();
        this.leftPivotPIDController2 = this.lpm2.getPIDController();
        this.rightPivotPIDController1 = this.rpm1.getPIDController();
        this.rightPivotPIDController2 = this.rpm2.getPIDController();

        this.feedforward = new Feedforward(0.0, 0.0, 0.0);
        
        this.leftPIDController1.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        this.leftPIDController2.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        this.rightPIDController1.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        this.rightPIDController2.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        
        this.leftPIDController1.setFeedbackDevice(this.leftEncoder1);
        this.leftPIDController2.setFeedbackDevice(this.leftEncoder2);
        this.rightPIDController1.setFeedbackDevice(this.rightEncoder1);
        this.rightPIDController2.setFeedbackDevice(this.rightEncoder2);

        if(reader != null)
            this.dblPivotCountPerDegree = reader.getSetting("PivotMotorCountsPerDegree", Constants.dblDefaultPiviotMotorCountsPerDegree);
        else
            this.dblPivotCountPerDegree = (4096.0 * 8.53) * 360.0; //TODO fix this
    }

    public void setLeft1Turret(double pos) {
        double position1 = this.leftEncoder1.getPosition() - (pos * this.dblPivotCountPerDegree);
        this.leftPivotPIDController1.setReference(position1, ControlType.kPosition);
    }
    public void setLeft2Turret(double pos) {
        double position1 = this.leftEncoder2.getPosition() - (pos * this.dblPivotCountPerDegree);
        this.leftPivotPIDController2.setReference(position1, ControlType.kPosition);
    }
    public void setRight1Turret(double pos) {
        double position1 = this.rightEncoder1.getPosition() - (pos * this.dblPivotCountPerDegree);
        this.rightPivotPIDController1.setReference(position1, ControlType.kPosition);
    }
    public void setRight2Turret(double pos) {
        double position1 = this.rightEncoder2.getPosition() - (pos * this.dblPivotCountPerDegree);
        this.rightPivotPIDController2.setReference(position1, ControlType.kPosition);
    }

    public void strafeDriveV1(double x1, double y1, double x2, double axleDistance, double wheelBase) {
        double r = Math.sqrt((axleDistance * axleDistance) + (wheelBase * wheelBase)); //Distance between adjectent wheel
        y1 *= -1;
    
        double a = x1 - x2 * (axleDistance / r); // x2 * (axleDistance / r) is the ratio of wheel distance from other wheels
        double b = x1 + x2 * (axleDistance / r);
        double c = y1 - x2 * (wheelBase / r);
        double d = y1 + x2 * (wheelBase / r);
    
        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));
        
        double backRightAngle = Constants.fromHeadingTo360((Math.atan2(a, d) / 3.14159) * 180);
        double backLeftAngle = Constants.fromHeadingTo360((Math.atan2(a, c) / 3.14159) * 180);
        double frontRightAngle = Constants.fromHeadingTo360((Math.atan2(b, d) / 3.14159) * 180);
        double frontLeftAngle = Constants.fromHeadingTo360((Math.atan2(b, c) / 3.14159) * 180);

        setLeft1Turret(frontLeftAngle);
        setLeft2Turret(backLeftAngle);
        setRight1Turret(frontRightAngle);
        setRight2Turret(backRightAngle);

        setLeft1Power(frontLeftSpeed);
        setLeft2Power(backLeftSpeed);
        setRight1Power(frontRightSpeed);
        setRight2Power(backRightSpeed);
        
        // System.out.println("LM1: " + lm1.get());
        // System.out.println("LM2: " + lm2.get());
        // System.out.println("RM1: " + rm1.get());
        // System.out.println("RM2: " + rm2.get());
    }

    public void normalizeLeftWheel() {
	    double pos = ((getLeft1PivotMotorPosition() + getLeft2PivotMotorPosition()) / 2) / this.dblPivotCountPerDegree;
	    setLeft1Turret(pos);
	    setLeft2Turret(pos);
    }
    public void normalizeRightWheel() {
        double pos = ((getRight1PivotMotorPosition() + getRight2PivotMotorPosition()) / 2) / this.dblPivotCountPerDegree;
        setRight1Turret(pos);
        setRight2Turret(pos);
    }
    public void normalizeAllWheel() {
	    double pos = ((getLeft1PivotMotorPosition() + getLeft2PivotMotorPosition() +
                getRight1PivotMotorPosition() + getRight2PivotMotorPosition()) / 4) * this.dblPivotCountPerDegree;
	    setLeft1Turret(pos);
	    setLeft2Turret(pos);
	    setRight1Turret(pos);
	    setRight2Turret(pos);
    }

    public double getLeft1PivotMotorPosition() {  return leftPivotEncoder1.getPosition(); }
    public double getLeft2PivotMotorPosition() {  return leftPivotEncoder2.getPosition(); }
    public double getRight1PivotMotorPosition() { return rightPivotEncoder1.getPosition(); }
    public double getRight2PivotMotorPosition() { return rightPivotEncoder2.getPosition(); }

	@Override
	public void setLeft1Power(double leftM1) { lm1.set(leftM1); }
	@Override
	public void setLeft2Power(double leftM2) { lm2.set(leftM2); }
	@Override
	public void setRight1Power(double rightM1) { rm1.set(rightM1); }
	@Override
	public void setRight2Power(double rightM2) { rm2.set(rightM2); }


    @Override
    public double getLeftMotor1Position() { return leftEncoder1.getPosition(); }
    @Override
    public double getLeftMotor2Position() { return leftEncoder2.getPosition(); }
    @Override
    public double getRightMotor1Position() { return rightEncoder1.getPosition(); }
    @Override
    public double getRightMotor2Position() { return rightEncoder2.getPosition(); }

    @Override
	public double getLeftMotorPosition() {
		return (leftEncoder1.getPosition() + leftEncoder2.getPosition()) / 2;
	}
	@Override
	public double getRightMotorPosition() {
		return (rightEncoder1.getPosition() + rightEncoder2.getPosition()) / 2;
	}

    @Override
    public void setLeftMotor1TargetPosition(double target) { this.dblLeftMotor1TargetPosition = target; }
    @Override
    public void setLeftMotor2TargetPosition(double target) { this.dblLeftMotor2TargetPosition = target; }
    @Override
    public void setRightMotor1TargetPosition(double target) { this.dblRightMotor1TargetPosition = target; }
    @Override
    public void setRightMotor2TargetPosition(double target) { this.dblRightMotor2TargetPosition = target; }

    @Override
    public double getLeftMotor1TargetPosition() { return this.dblLeftMotor1TargetPosition; }
    @Override
    public double getLeftMotor2TargetPosition() { return this.dblLeftMotor2TargetPosition; }
    @Override
    public double getRightMotor1TargetPosition() { return this.dblRightMotor1TargetPosition; }
    @Override
    public double getRightMotor2TargetPosition() { return this.dblRightMotor2TargetPosition; }
}