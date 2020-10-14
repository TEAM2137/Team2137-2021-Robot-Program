package frc.robot;

public class Constants {
    public static final int intLM1ID = 0;
    public static final int intLM2ID = 1;
    public static final int intRM1ID = 2;
    public static final int intRM2ID = 3;
    public static final int intSolenoidID = 0;

    public static final boolean boolLM1Invert = false;
    public static final boolean boolLM2Invert = false;
    public static final boolean boolRM1Invert = false;
    public static final boolean boolRM2Invert = false;

    public static final double dblMaxBaseSpeed = 1.0; // 1.0 Max & 0.0 Min
    public static final double dblMinBaseSpeed = 0.0; // 0.0 Min & 1.0 Max
    
    public static final double dblBaseP = 0.0;
    public static final double dblBaseI = 0.0;
    public static final double dblBaseD = 0.0;
    public static final double dblBaseS = 0.0;
    public static final double dblBaseV = 0.0;
    public static final double dblBaseA = 0.0;
   
    public static final double dblBaseEncoderCPR = 4096.0; // The counts per rotation that the encoders have on the base
    public static final double dblLowGearMaxVelocity = 5000.0;
    public static final double dblHighHearMinVelocity = 5000.0;
    public static final double dblHighGearVoltageLimit = 10.0;

    /**
     * Clips the robot base speed with in the range by @link dblMaxBaseSpeed and @link dblMinBaseSpeed
     * @param input
     * @return
     */
    public static double clipBaseSpeed(double input){
        return clip(input, dblMinBaseSpeed, dblMaxBaseSpeed);
    }

    /**
     * Cuts the input number to the min and max
     * @param input Number
     * @param min Min value
     * @param max Max value
     */
    public static double clip(double input, double min, double max){
        if (input >= max) {
            return max;
        } else if (input <= min) {
            return min;
        } else {
            return input;
        }
    }
}