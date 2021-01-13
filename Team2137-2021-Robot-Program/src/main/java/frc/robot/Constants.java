package frc.robot;

import org.opencv.core.Point;
import frc.robot.util.Motor;

public class Constants {
    public static final int dblDefaultDriverControllerPort = 0;

    public static final int intDefaultLM1ID = 0;
    public static final int intDefaultLM2ID = 1;
    public static final int intDefaultRM1ID = 2;
    public static final int intDefaultRM2ID = 3;
    public static final int intSolenoidID = 0;
    public static final int intMaxLogFiles = 30;

    public static final boolean boolDefaultLM1Invert = false;
    public static final boolean boolDefaultLM2Invert = false;
    public static final boolean boolDefaultRM1Invert = false;
    public static final boolean boolDefaultRM2Invert = false;
    public static final boolean boolShifterInvert = false;

    public static final double dblMaxBaseSpeed = 1.0; // 1.0 Max & 0.0 Min
    public static final double dblMinBaseSpeed = 0.0; // 0.0 Min & 1.0 Max
    
    public static final double[] dblBaseP = {0.0, 0.0}; // Low, High
    public static final double[] dblBaseI = {0.0, 0.0};
    public static final double[] dblBaseD = {0.0, 0.0};
    public static final double[] dblBaseS = {0.0, 0.0};
    public static final double[] dblBaseV = {0.0, 0.0};
    public static final double[] dblBaseA = {0.0, 0.0};
   
    public static final double dblBaseEncoderCPR = 4096.0; // The counts per rotation that the encoders have on the base
    public static final double dblLowGearMaxVelocity = 5000.0;
    public static final double dblHighHearMinVelocity = 5000.0;
    public static final double dblHighGearVoltageLimit = 10.0;

    /**
     * Default Swerve Values
     */
    public static final double dblDefaultPiviotMotorCountsPerDegree = 0.0;


    public static final double dblDefaultRobotWheelBase = 0.0;
    public static final double dblDefaultRobotAxelTrack = 0.0;

    public static final double dblDerivativeAccuracyFactor = 0.000001;

    public static final String strLogDirectory = "\\home\\lvuser\\Logs\\";
    public static final String strStepFileDirectory = "\\home\\lvuser\\Steps\\";
    public static final String strSettingFileDirectory = "\\home\\lvuser\\Settings\\";

    /**
     * Clips the robot base speed with in the range by @link dblMaxBaseSpeed and @link dblMinBaseSpeed
     * @param input
     * @return
     */
    public static double clipBaseSpeed(final double input){
        return clip(input, dblMinBaseSpeed, dblMaxBaseSpeed);
    }

    /**
     * Cuts the input number to the min and max
     * @param input Number
     * @param min Min value
     * @param max Max value
     */
    public static double clip(final double input, final double min, final double max) {
        if (input >= max) {
            return max;
        } else if (input <= min) {
            return min;
        } else {
            return input;
        }
    }

    public static boolean withinClip(double value, double goal, double width) {
        return value <= goal + width && value >= goal - width;
    }

    public static Point[] translateCenterToWheelPoints(Point centerPoint, double steeringWidth, double steeringBase) {
        Point leftFrontPoint = new Point(centerPoint.x - (steeringWidth / 2), centerPoint.y + (steeringBase / 2));
        Point leftBackPoint = new Point(centerPoint.x - (steeringWidth / 2), centerPoint.y - (steeringBase / 2));
        Point rightFrontPoint = new Point(centerPoint.x + (steeringWidth / 2), centerPoint.y + (steeringBase / 2));
        Point rightBackPoint = new Point(centerPoint.x + (steeringWidth / 2), centerPoint.y - (steeringBase / 2));
        Point[] tmp = {leftFrontPoint, leftBackPoint, rightFrontPoint, rightBackPoint};
        return tmp;
    }

    public static double[] translateWheelAnglePositive(Point piviotPoint, Point positionPoint, double steeringWidth, double steeringBase) {
        Point[] wheelPoints = translateCenterToWheelPoints(positionPoint, steeringWidth, steeringBase);

        double frontBaseOriginDifferance = wheelPoints[0].y - piviotPoint.y; //C1 = FrontLeftWheelY - PiviotPointY
        double backWidthOriginDifferance = wheelPoints[3].y - piviotPoint.y; //C2 = BackRightWheelX - PiviotPointX

        double xDifferenceCenterOrigin = positionPoint.x - piviotPoint.x;

        double left1WheelAngle = Math.atan((frontBaseOriginDifferance) / (xDifferenceCenterOrigin - (steeringWidth / 2)));
        double left2WheelAngle = Math.atan((backWidthOriginDifferance) / (xDifferenceCenterOrigin - (steeringWidth / 2)));
        double right1WheelAngle = Math.atan((frontBaseOriginDifferance) / (xDifferenceCenterOrigin + (steeringWidth / 2)));
        double right2WheelAngle = Math.atan((backWidthOriginDifferance) / (xDifferenceCenterOrigin + (steeringWidth / 2)));
        double[] tmp = {left1WheelAngle, left2WheelAngle, right1WheelAngle, right2WheelAngle};

        return tmp;
    }

    public static double fromHeadingTo360(double a) {
        return a < 0 ? a + 360 : a;
    }

    //      _____________      _______
    //     / 2          2     / 2    2
    //   \/ r  - (x + h)  - \/ r  - x 
    // m = -----------------------------
    //                 h              
    public static double pointDerivativeHalfCircle(double radius, double x, double accuracyFactor, boolean invert) {
        if (invert)
            return ((Math.sqrt(Math.pow(radius, 2) - Math.pow(x + accuracyFactor, 2)) * -1) + Math.sqrt(Math.pow(radius, 2) - Math.pow(x, 2))) / accuracyFactor;
        else
            return (Math.sqrt(Math.pow(radius, 2) - Math.pow(x + accuracyFactor, 2)) - Math.sqrt(Math.pow(radius, 2) - Math.pow(x, 2))) / accuracyFactor;
    }

    public enum MotorTypes {
        NEO ("NEO"),
        FALCON ("FALCON"),
        CIM ("CIM"),
        MINICIM ("MiniCIM"),
        BAG ("BAG");

        String name = "";

        MotorTypes (String str) {
            this.name = str;
        }

        public String toString() {
            return this.name;
        }
    }

    public enum EncoderTypes {
        CTRE_MAG_ABS ("CTRE MAG ABS"),
        REV_BORE ("REV BORE"),
        NEO_MAG ("NEO MAG");

        String name = "";

        EncoderTypes (String str) {
            this.name = str;
        }

        public String toString() {
            return this.name;
        }
    }

    public enum StepState {
        STATE_INIT ("STATE_INIT"),
        STATE_START ("STATE_START"),
        STATE_RUNNING ("STATE_RUNNING"),
        STATE_PAUSE ("STATE_PAUSE"),
        STATE_COMPLETE ("STATE_COMPLETE"),
        STATE_TIMEOUT ("STATE_TIMEOUT"),
        STATE_ERROR ("STATE_ERROR"),
        STATE_FINISHED ("STATE_FINISHED");

        public String name = "";

        StepState (String _name) {
            this.name = _name;
        }

        public String toString() {
            return this.name;
        }
    }
}