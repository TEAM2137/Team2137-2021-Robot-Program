package frc.robot.base;

public abstract class HardwareDriveTrain {
    public enum ShifterStates {
        High,
        Low
    }

    public abstract void init();
}