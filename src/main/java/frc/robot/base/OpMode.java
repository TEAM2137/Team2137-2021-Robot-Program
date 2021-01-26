package frc.robot.base;

public abstract class OpMode extends LinearOpMode {

    @Override
    public void run() {
        init();

        while (OpModeIsActive()) {
            loop();
        }

        exit();
    }

    /**
     * Override the function and the code only run once
     */
    public abstract void init();
    /**
     * Override the function and the code runs repeatedly
     */
    public abstract void loop();
    /**
     * Override the function and when exiting the loop runs
     */
    public abstract void exit();

}