package frc.robot;

import javax.management.RuntimeErrorException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.base.Gamepad;
import frc.robot.base.OpMode;
import frc.robot.base.Gamepad.Axis;
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
	public void init(boolean test) {
		//this.settingReader = new XMLSettingReader("RobotSetting");
		this.driveTrain = new BaseSwerve();
		this.driveTrain.init(null, null);

		//driverController = new Gamepad((int) (settingReader.getSetting("DriverControllerPort", Constants.dblDefaultDriverControllerPort)));
		this.driverController = new Gamepad(0);

		//dblWheelBase = settingReader.getSetting("RobotWheelBase", Constants.dblDefaultRobotWheelBase);
		//dblWheelTrack = settingReader.getSetting("RobotAxelTrack", Constants.dblDefaultRobotAxelTrack);
		this.dblWheelBase = 36;
		this.dblWheelTrack = 48;
	}

	@Override
	public void loop(boolean test) {
		//System.out.println(driverController.getRawAxis(Axis.kLeftX.getPort()));
        driveTrain.strafeDriveV1(driverController.getRawAxis(Axis.kLeftX.getPort()), driverController.getRawAxis(Axis.kLeftY.getPort()), driverController.getRawAxis(Axis.kRightX.getPort()),  this.dblWheelTrack, this.dblWheelBase);
	}

	@Override
	public void end(boolean test) {
		
	}
}