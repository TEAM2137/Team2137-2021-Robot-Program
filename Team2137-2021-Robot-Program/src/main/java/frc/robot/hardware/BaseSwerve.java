package frc.robot.hardware;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.base.Feedforward;
import frc.robot.base.HardwareDriveTrain;
import frc.robot.util.FileLogger;
import frc.robot.util.Motor;
import frc.robot.util.XMLSettingReader;

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

    private CANEncoder leftPiviotEncoder1;
    private CANEncoder leftPiviotEncoder2;
    private CANEncoder rightPiviotEncoder1;
    private CANEncoder rightPiviotEncoder2;

    private CANPIDController leftPIDController1;
    private CANPIDController leftPIDController2;
    private CANPIDController rightPIDController1;
    private CANPIDController rightPIDController2;

    private CANPIDController leftPiviotPIDController1;
    private CANPIDController leftPiviotPIDController2;
    private CANPIDController rightPiviotPIDController1;
    private CANPIDController rightPiviotPIDController2;

    private double dblPivotCountPerDegree = 0;
    private double dblLeftMotor1TargetPosition = 0;
    private double dblLeftMotor2TargetPosition = 0;
    private double dblRightMotor1TargetPosition = 0;
    private double dblRightMotor2TargetPosition = 0;

    private Feedforward feedforward;
    
	@Override
	public void init(XMLSettingReader reader, FileLogger log) {
        this.dblPivotCountPerDegree = Double.parseDouble(reader.getSetting("PiviotMotorCountsPerDegree"));

        Motor tmpLM1 = reader.getMotor("LM1");
        Motor tmpLM2 = reader.getMotor("LM2");
        Motor tmpRM1 = reader.getMotor("RM1");
        Motor tmpRM2 = reader.getMotor("RM2");

        Motor tmpLPM1 = reader.getMotor("LPM1");
        Motor tmpLPM2 = reader.getMotor("LPM2");
        Motor tmpRPM1 = reader.getMotor("RPM1");
        Motor tmpRPM2 = reader.getMotor("RPM2");

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

        this.leftEncoder1 = new CANEncoder(this.lm1, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);
        this.leftEncoder2 = new CANEncoder(this.lm2, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);
        this.rightEncoder1 = new CANEncoder(this.rm1, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);
        this.rightEncoder2 = new CANEncoder(this.rm2, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);

        this.leftPiviotEncoder1 = new CANEncoder(this.lm1, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);
        this.leftPiviotEncoder2 = new CANEncoder(this.lm2, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);
        this.rightPiviotEncoder1 = new CANEncoder(this.rm1, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);
        this.rightPiviotEncoder2 = new CANEncoder(this.rm2, EncoderType.kQuadrature, (int) Constants.dblBaseEncoderCPR);

        this.leftPIDController1 = this.lm1.getPIDController();
        this.leftPIDController2 = this.lm2.getPIDController();
        this.rightPIDController1 = this.rm1.getPIDController();
        this.rightPIDController2 = this.rm2.getPIDController();
        
        this.leftPiviotPIDController1 = this.lpm1.getPIDController();
        this.leftPiviotPIDController1 = this.lpm2.getPIDController();
        this.leftPiviotPIDController1 = this.rpm1.getPIDController();
        this.leftPiviotPIDController1 = this.rpm2.getPIDController();

        this.feedforward = new Feedforward(0.0, 0.0, 0.0);
        
        this.leftPIDController1.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        this.leftPIDController2.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        this.rightPIDController1.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        this.rightPIDController2.setOutputRange(Constants.dblMinBaseSpeed, Constants.dblMaxBaseSpeed);
        
        this.leftPIDController1.setFeedbackDevice(this.leftEncoder1);
        this.leftPIDController2.setFeedbackDevice(this.leftEncoder2);
        this.rightPIDController1.setFeedbackDevice(this.rightEncoder1);
        this.rightPIDController2.setFeedbackDevice(this.rightEncoder2);

        this.dblPivotCountPerDegree = reader.getSetting("PiviotMotorCountsPerDegree", Constants.dblDefaultPiviotMotorCountsPerDegree);
    }

    public void setLeft1Turret(double pos) {
        double position1 = this.leftEncoder1.getPosition() - (pos * this.dblPivotCountPerDegree);   
        this.leftPiviotPIDController1.setReference(position1, ControlType.kPosition);
    }
    public void setLeft2Turret(double pos) {
        double position1 = this.leftEncoder2.getPosition() - (pos * this.dblPivotCountPerDegree);   
        this.leftPiviotPIDController2.setReference(position1, ControlType.kPosition);
    }
    public void setRight1Turret(double pos) {
        double position1 = this.rightEncoder1.getPosition() - (pos * this.dblPivotCountPerDegree);   
        this.rightPiviotPIDController1.setReference(position1, ControlType.kPosition);
    }
    public void setRight2Turret(double pos) {
        double position1 = this.rightEncoder2.getPosition() - (pos * this.dblPivotCountPerDegree);   
        this.rightPiviotPIDController2.setReference(position1, ControlType.kPosition);
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

        setLeftTurret(frontLeftAngle, backLeftAngle);
        setRightTurret(frontRightAngle, backRightAngle);
        
        setLeft1Power(frontLeftSpeed);
        setLeft2Power(backLeftSpeed);
        setRight1Power(frontRightSpeed);
        setRight2Power(backRightSpeed);
    }

    public void normalizeLeftWheel() {
	    double pos = ((getLeft1PiviotMotorPosition() + getLeft2PiviotMotorPosition()) / 2) / this.dblPivotCountPerDegree;
	    setLeftTurret(pos, pos);
    }
    public void normalizeRightWheel() {
        double pos = ((getRight1PiviotMotorPosition() + getRight2PiviotMotorPosition()) / 2) / this.dblPivotCountPerDegree;
        setLeftTurret(pos, pos);
    }
    public void normalizeAllWheel() {
	    double pos = ((getLeft1PiviotMotorPosition() + getLeft2PiviotMotorPosition() +
                getRight1PiviotMotorPosition() + getRight2PiviotMotorPosition()) / 4) * this.dblPivotCountPerDegree;
	    setLeftTurret(pos, pos);
	    setRightTurret(pos, pos);
    }

    public double getLeft1PiviotMotorPosition() {  return leftPiviotEncoder1.getPosition(); }
    public double getLeft2PiviotMotorPosition() {  return leftPiviotEncoder2.getPosition(); }
    public double getRight1PiviotMotorPosition() { return rightPiviotEncoder1.getPosition(); }
    public double getRight2PiviotMotorPosition() { return rightPiviotEncoder2.getPosition(); }

	@Override
	public void setLeft1Power(double leftM1) { lm1.set(Constants.clipBaseSpeed(leftM1)); }
	@Override
	public void setLeft2Power(double leftM2) { lm2.set(Constants.clipBaseSpeed(leftM2)); }
	@Override
	public void setRight1Power(double rightM1) { rm1.set(Constants.clipBaseSpeed(rightM1)); }
	@Override
	public void setRight2Power(double rightM2) { rm2.set(Constants.clipBaseSpeed(rightM2)); }


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