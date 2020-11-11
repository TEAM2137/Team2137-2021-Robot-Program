package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.base.OpMode;
import frc.robot.util.FileLogger;
import frc.robot.util.XMLStepReader;
import frc.robot.util.FileLogger.EventType;

public class Autonomous extends OpMode {

    public FileLogger logger        = null;
    public DriverStation mDS        = null;
    public XMLStepReader mStepFile  = null;
    
    public int mintDebugLevel       = 0;

	@Override
	public void init() {
        this.mDS = DriverStation.getInstance();

        this.mintDebugLevel = (int) SmartDashboard.getNumber("debug", 0);

        logger = new FileLogger(0);
        logger.writeEvent(0, EventType.Status, "Starting FileLog...");
    
        logger.writeEvent(8, EventType.Debug, "Opening XML Step File...");
        mStepFile = new XMLStepReader(mDS.getAlliance() + String.valueOf(mDS.getLocation()));

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
}