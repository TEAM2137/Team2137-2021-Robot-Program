package frc.robot;

public class Constants {
    public static int mintLM1ID = 0;
    public static int mintLM2ID = 1;
    public static int mintRM1ID = 2;
    public static int mintRM2ID = 3;

    public static boolean boolLM1Invert = false;
    public static boolean boolLM2Invert = false;
    public static boolean boolRM1Invert = false;
    public static boolean boolRM2Invert = false;

    public static double dblMaxBaseSpeed = 1.0; // 1.0 Max & 0.0 Min
    public static double dblMinBaseSpeed = 0.0; // 0.0 Min & 1.0 Max
    public static double dblBaseP = 0.0;
    public static double dblBaseI = 0.0;
    public static double dblBaseD = 0.0;
    public static double dblBaseS = 0.0;
    public static double dblBaseV = 0.0;
    public static double dblBaseA = 0.0;
    public static double dblBaseEncoderCPR = 4096.0; // The counts per rotation that the encoders have on the base
    public static double dblLowGearMaxVelocity = 5000.0;
    public static double dblHighHearMinVelocity = 5000.0;

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