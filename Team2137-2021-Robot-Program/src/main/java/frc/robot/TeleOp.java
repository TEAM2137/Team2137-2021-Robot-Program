package frc.robot;

import javax.management.RuntimeErrorException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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

    private XboxController driverController = null;

	@Override
	public void init(boolean test) {
		//this.settingReader = new XMLSettingReader("RobotSetting");
		this.driveTrain = new BaseSwerve();
		this.driveTrain.init(null, null);

		//driverController = new Gamepad((int) (settingReader.getSetting("DriverControllerPort", Constants.dblDefaultDriverControllerPort)));
		this.driverController = new XboxController(0);

		//dblWheelBase = settingReader.getSetting("RobotWheelBase", Constants.dblDefaultRobotWheelBase);
		//dblWheelTrack = settingReader.getSetting("RobotAxelTrack", Constants.dblDefaultRobotAxelTrack);
		this.dblWheelBase = 36;
		this.dblWheelTrack = 48;
	}

	@Override
	public void loop(boolean test) {
		DriverStation.reportError("IN LOOP", false);
		System.out.println(driverController.getX(Hand.kLeft));
        driveTrain.strafeDriveV1(driverController.getX(GenericHID.Hand.kLeft), driverController.getY(GenericHID.Hand.kLeft), driverController.getX(GenericHID.Hand.kRight),  this.dblWheelTrack, this.dblWheelBase);
	}

	@Override
	public void end(boolean test) {
		
	}
}