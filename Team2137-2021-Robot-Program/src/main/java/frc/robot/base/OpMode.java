package frc.robot.base;

public abstract class OpMode extends LinearOpMode {
    @Override
    public void run(boolean inTest){
        init();

        while(this.OpModeActive() && !Thread.currentThread().isInterrupted()) {
            loop();
        }

        end();
    }

    /**
     * Init is run before loop and only runs once.
     * This is where varables and other processes are initalized.
     */
    public abstract void init();

    /**
     * Loop is run before end and loops again until OpModeActive returns false.
     * This is where the bulk of the program is executed.
     */
    public abstract void loop();

    /**
     * End is the last thing to run. This is stopping processes and returning.
     * This should not leave Threads and other programs running.
     */
    public abstract void end();
}