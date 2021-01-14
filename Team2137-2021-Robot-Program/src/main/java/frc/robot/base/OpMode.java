package frc.robot.base;

public abstract class OpMode {
    /**
     * Init is run before loop and only runs once.
     * This is where varables and other processes are initalized.
     */
    public abstract void init(boolean test);

    /**
     * Loop is run before end and loops again until OpModeActive returns false.
     * This is where the bulk of the program is executed.
     */
    public abstract void loop(boolean test);

    /**
     * End is the last thing to run. This is stopping processes and returning.
     * This should not leave Threads and other programs running.
     */
    public abstract void end(boolean test);
}