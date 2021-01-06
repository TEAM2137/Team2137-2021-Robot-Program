package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.base.Gamepad;
import frc.robot.base.OpMode;
import frc.robot.hardware.Base6Wheel;
import frc.robot.hardware.BaseSwerve;
import frc.robot.util.XMLSettingReader;

public class TeleOp extends OpMode {

    private BaseSwerve driveTrain;
    private XMLSettingReader settingReader;

    private double dblWheelBase = 0;
    private double dblWheelTrack = 0;

    private Gamepad driverController = null;

	@Override
	public void init() {
		this.settingReader = new XMLSettingReader("RobotSetting");
		this.driveTrain = new BaseSwerve();

		driverController = new Gamepad((int) (settingReader.getSetting("DriverControllerPort", Constants.dblDefaultDriverControllerPort)));

		dblWheelBase = settingReader.getSetting("RobotWheelBase", Constants.dblDefaultRobotWheelBase);
		dblWheelTrack = settingReader.getSetting("RobotAxelTrack", Constants.dblDefaultRobotAxelTrack);
	}

	@Override
	public void loop() {
        driveTrain.strafeDriveV1(driverController.getX(GenericHID.Hand.kLeft), driverController.getY(GenericHID.Hand.kLeft), driverController.getX(GenericHID.Hand.kRight),  this.dblWheelTrack, this.dblWheelBase);
	}

	@Override
	public void end() {
		
	}
}