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

    public StepState mStateBangBangDrive            = StepState.STATE_FINSIHED;
    public StepState mStatePIDDrive                 = StepState.STATE_FINSIHED;
    public StepState mStateDifferentialTurning      = StepState.STATE_FINSIHED;

    public double dblLeftWheelTargetPosition = 0;
    public double dblRightWheelTargetPosition = 0;
    
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
            if (v == StepState.STATE_FINSIHED) {
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
            case "Piviot":
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
                    this.mStatePIDDrive = StepState.STATE_FINSIHED;
                }
                if (Constants.withinClip(this.mDriveTrain.getRightMotorPosition(), this.dblRightWheelTargetPosition, step.getParm(0,0.0))) {
                    this.mStatePIDDrive = StepState.STATE_FINSIHED;
                }
                break;
            case STATE_FINSIHED:
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

                if (leftSide && rightSide) mStateBangBangDrive = StepState.STATE_FINSIHED;
            case STATE_FINSIHED:
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
                    mStateDifferentialTurning = StepState.STATE_FINSIHED;
                }
                break;
            case STATE_FINSIHED:
                break;
        }
    }
}