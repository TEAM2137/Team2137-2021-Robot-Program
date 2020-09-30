package frc.robot.base;

public abstract class LinearOpMode {

    public abstract void run(boolean inTest);
    
    public boolean OpModeActive(){
        return RobotCommander.boolRobotActive;
    }
}