package frc.robot.base;

public abstract class LinearOpMode extends Thread {

    private volatile boolean opModeActive = true;
    private volatile boolean isInTest = false;

    //public abstract void run();

    public boolean OpModeIsActive() {
        return this.opModeActive;
    }

    public boolean isTest() {
        return this.isInTest;
    }

    public void EndOpMode() {
        this.opModeActive = false;
    }
}
