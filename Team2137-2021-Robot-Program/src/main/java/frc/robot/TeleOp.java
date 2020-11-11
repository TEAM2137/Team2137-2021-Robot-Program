package frc.robot;

import frc.robot.base.OpMode;
import frc.robot.hardware.Base6Wheel;

public class TeleOp extends OpMode {

    private Base6Wheel driveTrain;

	@Override
	public void init() {
		this.driveTrain = new Base6Wheel();
	}

	@Override
	public void loop() {
        
	}

	@Override
	public void end() {
		
	}
}