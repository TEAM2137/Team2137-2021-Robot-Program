package frc.robot;

import java.util.HashMap;
import java.util.function.Consumer;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.StepState;
import frc.robot.base.HardwareDriveTrain;
import frc.robot.base.OpMode;
import frc.robot.hardware.Base6Wheel;
import frc.robot.hardware.BaseSwerve;
import frc.robot.util.XMLSettingReader;
import frc.robot.util.XMLStepReader;
import frc.robot.util.FileLogger;
import frc.robot.util.Step;
import frc.robot.util.FileLogger.EventType;

public class Autonomous extends OpMode {

    public FileLogger logger                = null;
    public DriverStation mDS                = null;
    public XMLStepReader mStepFile          = null;
    public XMLSettingReader mSettingFile    = null;

    public StepState mStateBangBangDrive            = StepState.STATE_FINISHED;
    public StepState mStatePIDDrive                 = StepState.STATE_FINISHED;
    public StepState mStateDifferentialTurning      = StepState.STATE_FINISHED;
    public StepState mStatePivotTurn                = StepState.STATE_FINISHED;

    private double dblSwerveLPM1RadiiFromPoint = 0.0;
    private double dblSwerveLPM2RadiiFromPoint = 0.0;
    private double dblSwerveRPM1RadiiFromPoint = 0.0;
    private double dblSwerveRPM2RadiiFromPoint = 0.0;

    private double dblLeftMotor1StartPosition = 0.0;
    private double dblLeftMotor2StartPosition = 0.0;
    private double dblRightMotor1StartPosition = 0.0;
    private double dblRightMotor2StartPosition = 0.0;

    public HardwareDriveTrain mDriveTrain = null;
    public Base6Wheel mBase6Wheel = null;
    public BaseSwerve mBaseSwerve = null;

    public int mintDebugLevel       = 0;

    public HashMap<Consumer<Void>, StepState> mActiveCommandList = new HashMap<Consumer<Void>, StepState>();

	@Override
	public void init() {
        this.mDS = DriverStation.getInstance();

        this.mintDebugLevel = (int) SmartDashboard.getNumber("debug", 0);

        logger = new FileLogger(0);
        logger.writeEvent(0, EventType.Status, "Starting FileLog...");
    
        logger.writeEvent(8, EventType.Debug, "Opening XML Step File...");
        mStepFile = new XMLStepReader(mDS.getAlliance() + String.valueOf(mDS.getLocation()));

        logger.writeEvent(8, EventType.Debug, "Opening XML Setting File...");
        mSettingFile = new XMLSettingReader("RobotSetting");

        mStepFile.forEachStep(E -> {
            logger.writeEvent(0, EventType.Debug, "Running Step" );
        });

    }

	@Override
	public void loop() {
		
	}

	@Override
	public void end() {
		
    }

    public void callActiveCommands() {
        this.mActiveCommandList.forEach((e, v) -> {
            if (v == StepState.STATE_FINISHED) {
                mActiveCommandList.remove(e);
            }
            e.accept(null);
        });
    }
    
    public void getCommand(String name, Step step) {
        logger.writeEvent(0, "Adding Active Command", name);
        switch (name) {
            case "BangBang":
                mStateBangBangDrive = StepState.STATE_INIT;
                this.mActiveCommandList.put((Void) -> {
                    BangBangDrive(step);
                }, mStateBangBangDrive);
                break;
            case "PIDDrive":
                mStatePIDDrive = StepState.STATE_INIT;
                this.mActiveCommandList.put((Void) -> {
                    BangBangDrive(step);
                }, mStatePIDDrive);
                break;
            case "Pivot":
                mStateDifferentialTurning = StepState.STATE_INIT;
                this.mActiveCommandList.put((Void) -> {
                    differentialTurning(step);
                }, mStateDifferentialTurning);
                break;

            default:
                logger.writeEvent(0, "Error Adding Active Command (Unknown)", name);
                break;
        }
    }

    public void PIDDrive(Step step) {
        switch(mStatePIDDrive){
            case STATE_INIT:
                this.mDriveTrain.addLeftMotorTargetPosition(step.getDistance());
                this.mDriveTrain.addRightMotorTargetPosition(step.getDistance());
                
                this.mStatePIDDrive = StepState.STATE_RUNNING;
                break;
            case STATE_RUNNING:
                this.mDriveTrain.setLeftPID(ControlType.kPosition, this.dblLeftWheelTargetPosition);
                this.mDriveTrain.setRightPID(ControlType.kPosition, this.dblRightWheelTargetPosition);
                
                if (Constants.withinClip(this.mDriveTrain.getLeftMotorPosition(), this.dblLeftWheelTargetPosition, step.getParm(0,0.0))) {
                    this.mStatePIDDrive = StepState.STATE_FINISHED;
                }
                if (Constants.withinClip(this.mDriveTrain.getRightMotorPosition(), this.dblRightWheelTargetPosition, step.getParm(0,0.0))) {
                    this.mStatePIDDrive = StepState.STATE_FINISHED;
                }
                break;
            case STATE_FINISHED:
                break;
        }
    }

    public void BangBangDrive(Step step) {
        switch(mStateBangBangDrive) {
            case STATE_INIT:
                this.mDriveTrain.addRightMotorTargetPosition(step.getDistance());
                this.mDriveTrain.addLeftMotorTargetPosition(step.getDistance());

                this.mStateBangBangDrive = StepState.STATE_RUNNING;
                break;
            case STATE_RUNNING:
                boolean leftSide = Constants.withinClip(this.mDriveTrain.getLeftMotorTargetPosition(), mDriveTrain.getLeftMotorPosition(), step.getParm(0, 0.0));
                boolean rightSide = Constants.withinClip(this.mDriveTrain.getRightMotorTargetPosition(), mDriveTrain.getRightMotorPosition(), step.getParm(0, 0.0));

                if(mDriveTrain.getLeftMotorPosition() < this.mDriveTrain.getLeftMotorTargetPosition() && !leftSide) {
                    mDriveTrain.setLeftPower(-1);
                } else if (mDriveTrain.getLeftMotorPosition() > this.mDriveTrain.getLeftMotorTargetPosition() && !leftSide) {
                    mDriveTrain.setLeftPower(1);
                }

                if (mDriveTrain.getRightMotorPosition() < this.mDriveTrain.getRightMotorTargetPosition() && !rightSide) {
                    mDriveTrain.setRightPower(-1);
                } else if (mDriveTrain.getRightMotorPosition() > this.mDriveTrain.getRightMotorTargetPosition() && !rightSide) {
                    mDriveTrain.setRightPower(1);
                }

                if (leftSide && rightSide) mStateBangBangDrive = StepState.STATE_FINISHED;
            case STATE_FINISHED:
                break;
        }
    }

    public void differentialTurning(Step step) {
        switch (mStateDifferentialTurning) {
            case STATE_INIT:
                double wheelbase;
                try {
                    wheelbase = Double.parseDouble(mSettingFile.getSetting("RobotWheelBase"));
                } catch (Exception e) {
                    wheelbase = Constants.dblDefaultRobotWheelBase;
                }

                double axelTrack;
                try {
                    axelTrack = Double.parseDouble(mSettingFile.getSetting("RobotAxelTrack"));
                } catch (Exception e) {
                    axelTrack = Constants.dblDefaultRobotAxelTrack;
                }

                double wheelRadius = Math.hypot((axelTrack / 2), (wheelbase / 2));
                double wheelBaseCircumference = Math.PI * (wheelRadius * 2);

                if (mBaseSwerve != null) {
                    mBaseSwerve.normalizeAllWheel();
                }

                mDriveTrain.addLeftMotor1TargetPosition(wheelBaseCircumference * (step.getDistance() / 360));
                mDriveTrain.addLeftMotor2TargetPosition(wheelBaseCircumference * (step.getDistance() / 360));
                mDriveTrain.addRightMotor1TargetPosition((wheelBaseCircumference * (step.getDistance() / 360)) * -1);
                mDriveTrain.addRightMotor2TargetPosition((wheelBaseCircumference * (step.getDistance() / 360)) * -1);

                mDriveTrain.setLeft1PID(ControlType.kPosition, mDriveTrain.getLeftMotor1TargetPosition());
                mDriveTrain.setLeft2PID(ControlType.kPosition, mDriveTrain.getLeftMotor2TargetPosition());
                mDriveTrain.setRight1PID(ControlType.kPosition, mDriveTrain.getRightMotor1TargetPosition());
                mDriveTrain.setRight2PID(ControlType.kPosition, mDriveTrain.getRightMotor2TargetPosition());

                mStateDifferentialTurning = StepState.STATE_RUNNING;
                break;
            case STATE_RUNNING:
                if (Constants.withinClip(mDriveTrain.getLeftMotor1Position(), mDriveTrain.getLeftMotor1TargetPosition(), step.getParm(0, 0.0))
                    && Constants.withinClip(mDriveTrain.getLeftMotor2Position(), mDriveTrain.getLeftMotor2TargetPosition(), step.getParm(0, 0.0))
                    && Constants.withinClip(mDriveTrain.getRightMotor1Position(), mDriveTrain.getRightMotor1TargetPosition(), step.getParm(0, 0.0))
                    && Constants.withinClip(mDriveTrain.getRightMotor2Position(), mDriveTrain.getRightMotor2TargetPosition(), step.getParm(0, 0.0))) {
                    mStateDifferentialTurning = StepState.STATE_FINISHED;
                }
                break;
            case STATE_FINISHED:
                break;
        }
    }

    /**
     * 0 --------- 2
     *       |
     *       |
     *       |
     * 1 --------- 3
     * @param step
     */
    public void PivotTurn (Step step) {
        switch(mStatePivotTurn) {
            case STATE_INIT:
                double wheelBase = mSettingFile.getSetting("RobotWheelBase", Constants.dblDefaultRobotWheelBase);
                double axelTrack = mSettingFile.getSetting("RobotAxelTrack", Constants.dblDefaultRobotAxelTrack);

                double wheelARadii = 0;
                double wheelBRadii = axelTrack / 2;
                double wheelCRadii = wheelBase / 2;
                double wheelDRadii = Math.hypot(axelTrack, wheelBase) / 2;

                double wheelA = wheelARadii * 2;
                double wheelB = (wheelBRadii * 2) * (step.getDistance() / 360); //Wheel on same axel
                double wheelC = ((wheelCRadii * 2) * Math.PI) * (step.getDistance() / 360); //Wheel on other axel but same side
                double wheelD = ((wheelDRadii * 2) * Math.PI) * (step.getDistance() / 360); //Far wheel

                switch (step.getParmInt(0)) {
                    case 0:
                        //Left Front Wheel Stationary
                        this.dblSwerveLPM1RadiiFromPoint = wheelARadii;
                        mDriveTrain.addLeftMotor1TargetPosition(wheelA);
                        this.dblSwerveLPM2RadiiFromPoint = wheelCRadii;
                        mDriveTrain.addLeftMotor2TargetPosition(wheelC);
                        this.dblSwerveRPM1RadiiFromPoint = wheelBRadii;
                        mDriveTrain.addRightMotor1TargetPosition(wheelB);
                        this.dblSwerveRPM2RadiiFromPoint = wheelDRadii;
                        mDriveTrain.addRightMotor2TargetPosition(wheelD);
                        break;
                    case 1:
                        //Left Back Wheel Stationary
                        this.dblSwerveLPM1RadiiFromPoint = wheelCRadii;
                        mDriveTrain.addLeftMotor1TargetPosition(wheelC);
                        this.dblSwerveLPM2RadiiFromPoint = wheelARadii;
                        mDriveTrain.addLeftMotor2TargetPosition(wheelA);
                        this.dblSwerveRPM1RadiiFromPoint = wheelDRadii;
                        mDriveTrain.addRightMotor1TargetPosition(wheelD);
                        this.dblSwerveRPM2RadiiFromPoint = wheelBRadii;
                        mDriveTrain.addRightMotor2TargetPosition(wheelB);
                        break;
                    case 2:
                        //Right Front Wheel Stationary
                        this.dblSwerveLPM1RadiiFromPoint = wheelBRadii;
                        mDriveTrain.addLeftMotor1TargetPosition(wheelB);
                        this.dblSwerveLPM2RadiiFromPoint = wheelDRadii;
                        mDriveTrain.addLeftMotor2TargetPosition(wheelD);
                        this.dblSwerveRPM1RadiiFromPoint = wheelARadii;
                        mDriveTrain.addRightMotor1TargetPosition(wheelA);
                        this.dblSwerveRPM2RadiiFromPoint = wheelCRadii;
                        mDriveTrain.addRightMotor2TargetPosition(wheelC);
                        break;
                    case 3:
                        //Right Back Wheel Stationary
                        this.dblSwerveLPM1RadiiFromPoint = wheelDRadii;
                        mDriveTrain.addLeftMotor1TargetPosition(wheelD);
                        this.dblSwerveLPM2RadiiFromPoint = wheelBRadii;
                        mDriveTrain.addLeftMotor2TargetPosition(wheelB);
                        this.dblSwerveRPM1RadiiFromPoint = wheelCRadii;
                        mDriveTrain.addRightMotor1TargetPosition(wheelC);
                        this.dblSwerveRPM2RadiiFromPoint = wheelDRadii;
                        mDriveTrain.addRightMotor2TargetPosition(wheelA);
                        break;
                }
                this.dblLeftMotor1StartPosition = mDriveTrain.getLeftMotor1Position();
                this.dblLeftMotor2StartPosition = mDriveTrain.getLeftMotor2Position();
                this.dblRightMotor1StartPosition = mDriveTrain.getRightMotor1Position();
                this.dblRightMotor2StartPosition = mDriveTrain.getRightMotor2Position();

                this.mStatePivotTurn = StepState.STATE_RUNNING;
                break;
            case STATE_RUNNING:
                double currentAngle = ((mDriveTrain.getLeftMotor1Position() + mDriveTrain.getLeftMotor2Position()
                        + mDriveTrain.getRightMotor1Position() + mDriveTrain.getRightMotor2Position()) / 4) /
                        ((this.dblLeftMotor1StartPosition + this.dblLeftMotor2StartPosition
                        + this.dblRightMotor1StartPosition + this.dblRightMotor2StartPosition) / 4);

                //@TODO fix the math only work with less than full circle
                double lm1X = Math.cos(Math.abs(this.dblLeftMotor1StartPosition - mDriveTrain.getLeftMotor1Position())/(this.dblSwerveLPM1RadiiFromPoint*2*Math.PI)) * this.dblSwerveLPM1RadiiFromPoint;
                double lm2X = Math.cos(Math.abs(this.dblLeftMotor2StartPosition - mDriveTrain.getLeftMotor2Position())/(this.dblSwerveLPM2RadiiFromPoint*2*Math.PI)) * this.dblSwerveLPM2RadiiFromPoint;
                double rm1X = Math.cos(Math.abs(this.dblRightMotor1StartPosition - mDriveTrain.getRightMotor1Position())/(this.dblSwerveRPM1RadiiFromPoint*2*Math.PI)) * this.dblSwerveRPM1RadiiFromPoint;
                double rm2X = Math.cos(Math.abs(this.dblRightMotor2StartPosition - mDriveTrain.getRightMotor2Position())/(this.dblSwerveRPM2RadiiFromPoint*2*Math.PI)) * this.dblSwerveRPM2RadiiFromPoint;

                if (Math.abs(currentAngle - step.getDistance()) < 90) {
                    mBaseSwerve.setLeft1Turret(Constants.pointDerivativeHalfCircle(this.dblSwerveLPM1RadiiFromPoint, lm1X, Constants.dblDerivativeAccuracyFactor, false) * 90);
                    mBaseSwerve.setLeft2Turret(Constants.pointDerivativeHalfCircle(this.dblSwerveLPM2RadiiFromPoint, lm2X, Constants.dblDerivativeAccuracyFactor, false) * 90);
                    mBaseSwerve.setRight1Turret(Constants.pointDerivativeHalfCircle(this.dblSwerveRPM1RadiiFromPoint, rm1X, Constants.dblDerivativeAccuracyFactor, false) * 90);
                    mBaseSwerve.setRight2Turret(Constants.pointDerivativeHalfCircle(this.dblSwerveRPM2RadiiFromPoint, rm2X, Constants.dblDerivativeAccuracyFactor, false) * 90);
                } else {
                    mBaseSwerve.setLeft1Turret(Constants.pointDerivativeHalfCircle(this.dblSwerveLPM1RadiiFromPoint, lm1X, Constants.dblDerivativeAccuracyFactor, true) * 90);
                    mBaseSwerve.setLeft2Turret(Constants.pointDerivativeHalfCircle(this.dblSwerveLPM2RadiiFromPoint, lm2X, Constants.dblDerivativeAccuracyFactor, true) * 90);
                    mBaseSwerve.setRight1Turret(Constants.pointDerivativeHalfCircle(this.dblSwerveRPM1RadiiFromPoint, rm1X, Constants.dblDerivativeAccuracyFactor, true) * 90);
                    mBaseSwerve.setRight2Turret(Constants.pointDerivativeHalfCircle(this.dblSwerveRPM2RadiiFromPoint, rm2X, Constants.dblDerivativeAccuracyFactor, true) * 90);
                }

                double tmpAMotorRadii = 0;

                switch (step.getParmInt(0)) {
                    case 0:
                        //Left Front Wheel Stationary
                        tmpAMotorRadii = this.dblSwerveRPM2RadiiFromPoint;
                        break;
                    case 1:
                        //Left Back Wheel Stationary
                        tmpAMotorRadii = this.dblSwerveRPM1RadiiFromPoint;
                        break;
                    case 2:
                        //Right Front Wheel Stationary
                        tmpAMotorRadii = this.dblSwerveLPM2RadiiFromPoint;
                        break;
                    case 3:
                        //Right Back Wheel Stationary
                        tmpAMotorRadii = this.dblSwerveLPM1RadiiFromPoint;
                        break;
                }

                //                    ((0.005 * r) * x)
                //y = (1 - (0.50 * r))
                //@link https://www.desmos.com/calculator/phrvovodjp
                double adjustedWheelSpeed = step.getDistance() * Math.pow(1 - (0.5 * step.getParm(3, 1.0)), (0.005 * step.getParm(3, 1.0)) * currentAngle);

                mBaseSwerve.setLeft1Power((this.dblSwerveLPM1RadiiFromPoint / tmpAMotorRadii) * adjustedWheelSpeed);
                mBaseSwerve.setLeft2Power((this.dblSwerveLPM2RadiiFromPoint / tmpAMotorRadii) * adjustedWheelSpeed);
                mBaseSwerve.setRight1Power((this.dblSwerveRPM1RadiiFromPoint / tmpAMotorRadii) * adjustedWheelSpeed);
                mBaseSwerve.setRight2Power((this.dblSwerveRPM2RadiiFromPoint / tmpAMotorRadii) * adjustedWheelSpeed);

                if(Constants.withinClip(currentAngle, step.getDistance(), step.getParm(1, 1.0))) {
                    mStatePivotTurn = StepState.STATE_FINISHED;
                }
                break;
            case STATE_FINISHED:
                break;
        }
    }
}