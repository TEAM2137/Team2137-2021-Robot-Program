package frc.robot.hardware;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import frc.robot.util.Constants;
import frc.robot.util.Motor;
import frc.robot.util.PID;
import org.opencv.core.Point;

import static frc.robot.util.Constants.toFieldRelativeChassisSpeeds;

public class SwerveBase {

    public SwerveModule leftFront;
    public SwerveModule leftBack;
    public SwerveModule rightFront;
    public SwerveModule rightBack;

    private double wheelBase = 0;
    private double axelTrack = 0;

    public SwerveBase (SwerveModule _leftFront, SwerveModule _leftBack, SwerveModule _rightFront, SwerveModule _rightBack, double _wheelBase, double _axelTrack) {
        leftFront = _leftFront;
        leftBack = _leftBack;
        rightFront = _rightFront;
        rightBack = _rightBack;
        this.wheelBase = _wheelBase;
        this.axelTrack = _axelTrack;
    }

    public void setAllSpeed(double speed) {
        leftFront.setDriveMotorSpeed(speed);
        leftBack.setDriveMotorSpeed(speed);
        rightFront.setDriveMotorSpeed(speed);
        rightBack.setDriveMotorSpeed(speed);
    }

    /**
     * Returns the speeds of the Swerve Drive Train when given the Controller Values
     * @param x1 - The left joystick -1 ~ 1
     * @param y1 - The left joystick -1 ~ 1
     * @param x2 - The rightjoy stick or turning button -1 ~ 1
     * @param axleDistance - Distance between the wheels on the same axel
     * @param wheelBase - Distance between different axles
     * @return An Array of Points with x being drive speed and y wheel angle in degree
     */
    public Point[] calculateSwerveMotorSpeeds(double x1, double y1, double x2, double axleDistance, double wheelBase) {
        double r = Math.sqrt((axleDistance * axleDistance) + (wheelBase * wheelBase)); //Distance between adjectent wheel
        y1 *= -1;

        double a = x1 - x2 * (axleDistance / r); // x2 * (axleDistance / r) is the ratio of wheel distance from other wheels
        double b = x1 + x2 * (axleDistance / r);
        double c = y1 - x2 * (wheelBase / r);
        double d = y1 + x2 * (wheelBase / r);

        Point backRight  = new Point();
        Point backLeft   = new Point();
        Point frontRight = new Point();
        Point frontLeft  = new Point();

        backRight.x     = Math.sqrt((a * a) + (d * d));
        backLeft.x      = Math.sqrt((a * a) + (c * c));
        frontRight.x    = Math.sqrt((b * b) + (d * d));
        frontLeft.x     = Math.sqrt((b * b) + (c * c));

        backRight.y     = Constants.fromHeadingTo360((Math.atan2(a, d) / 3.14159) * 180);
        backLeft.y      = Constants.fromHeadingTo360((Math.atan2(a, c) / 3.14159) * 180);
        frontRight.y    = Constants.fromHeadingTo360((Math.atan2(b, d) / 3.14159) * 180);
        frontLeft.y     = Constants.fromHeadingTo360((Math.atan2(b, c) / 3.14159) * 180);

        return new Point[]{frontLeft, backLeft, frontRight, backRight};
    }

    public void strafeDriveV1(double x1, double y1, double x2) {
        Point[] speeds = calculateSwerveMotorSpeeds(x1, y1, x2, axelTrack, wheelBase);

        this.leftFront.setTurnMotorPosition(speeds[0].y);
        this.leftBack.setTurnMotorPosition(speeds[1].y);
        this.rightFront.setTurnMotorPosition(speeds[2].y);
        this.rightBack.setTurnMotorPosition(speeds[3].y);

        this.leftFront.setDriveMotorSpeed(speeds[0].x);
        this.leftBack.setDriveMotorSpeed(speeds[1].x);
        this.rightFront.setDriveMotorSpeed(speeds[2].x);
        this.rightBack.setDriveMotorSpeed(speeds[3].x);

        // System.out.println("LM1: " + lm1.get());
        // System.out.println("LM2: " + lm2.get());
        // System.out.println("RM1: " + rm1.get());
        // System.out.println("RM2: " + rm2.get());
    }

    public void strafeDriveFieldCentric(double x1, double y1, double x2, Rotation2d robotAngle) {
        double[] translatedSpeeds = toFieldRelativeChassisSpeeds(x1, y1, x2, robotAngle);
        strafeDriveV1(translatedSpeeds[0], translatedSpeeds[1], translatedSpeeds[2]);
    }

    public void setLeftVelocity(double goal) {

    }

    public void setRightVelocity(double goal) {
        
    }
}
