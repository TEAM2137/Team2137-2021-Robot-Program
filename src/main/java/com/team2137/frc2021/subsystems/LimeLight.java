package com.team2137.frc2021.subsystems;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.util.PID;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.Point;

public class LimeLight extends SubsystemBase {

    public enum LimeLightValues {
        TX ("tx"),
        TY ("ty"),
        TA ("ta"),
        TV ("tv"),
        LEDMODE ("ledMode"),
        CAMMODE ("camMode"),
        PIPELINE ("pipeLine"),;

        public String tableName = "";

        LimeLightValues (String tableValue) {
            tableName = tableValue;
        }

        public String getTableName() {
            return tableName;
        }

        public double getValue(NetworkTable table) {
            return table.getEntry(tableName).getDouble(0.0);
        }
    }

    private NetworkTable limeLightTableShoot; //Network table for getting the LimeLight values
    private NetworkTable limeLightTableIntake;

    /**
     * tx - the degrees the target is on the camera
     * ty- the degrees the target is vertically on the camera
     * ta - the area of the bounding box in the total field of view
     */
    private double txShoot, tyShoot, taShoot; // Target position values
    private boolean tvShoot = false;

    private double txBall, tyBall, taBall;
    private boolean tvBall = false;

    private double cameraAngleShoot; //In Radians
    private double cameraAngleBall;

    private double cameraIntakeHypot;

    private Point targetFieldCentricPosition;
    private Point robotCentricCameraPosition;

    public LimeLight(double _cameraAngleShoot, Point _robotCentricCameraPosition, Point _feildCentricTargetPosition, PID pidValues) {
        limeLightTableShoot = NetworkTableInstance.getDefault().getTable("limelight");
        cameraAngleShoot = Math.toRadians(_cameraAngleShoot);
        robotCentricCameraPosition = _robotCentricCameraPosition;
    }

    /**
     * Create a new LimeLight object for the Subsystem using the default values in {@link com.team2137.frc2021.Constants}
     */
    public LimeLight() {
        limeLightTableShoot = NetworkTableInstance.getDefault().getTable("limelight");
        limeLightTableIntake = NetworkTableInstance.getDefault().getTable("test"); //TODO fix

        cameraAngleShoot = Math.toRadians(Constants.Shooter.LimeLightShootingCameraAngleDegree);
        cameraAngleBall = Math.toRadians(0);

        robotCentricCameraPosition = Constants.Shooter.LimeLightShootingCameraPosition;

        targetFieldCentricPosition = Constants.Shooter.LimeLightTargetFieldPosition;
    }

    @Override
    public void periodic() {
        txShoot = Math.toRadians(limeLightTableShoot.getEntry(LimeLightValues.TX.getTableName()).getDouble(0.0));
        tyShoot= Math.toRadians(limeLightTableShoot.getEntry(LimeLightValues.TY.getTableName()).getDouble(0.0));
        taShoot = limeLightTableShoot.getEntry(LimeLightValues.TA.getTableName()).getDouble(0.0);
        tvShoot = limeLightTableShoot.getEntry(LimeLightValues.TV.getTableName()).getDouble(0) >= 1.0;

        txBall = Math.toRadians(limeLightTableIntake.getEntry(LimeLightValues.TX.getTableName()).getDouble(0.0));
        tyBall = Math.toRadians(limeLightTableIntake.getEntry(LimeLightValues.TY.getTableName()).getDouble(0.0));
        taBall = limeLightTableIntake.getEntry(LimeLightValues.TA.getTableName()).getDouble(0.0);
        tvBall = limeLightTableIntake.getEntry(LimeLightValues.TV.getTableName()).getDouble(0) >= 1.0;

        SmartDashboard.putNumber("Robot X: ", getCameraXPosition());
    }

    /**
     * Find the radius of the arc of the robot to the target or the distance to the target
     * @return Camera distance from object on horizontal plane
     */
    public double getRobotRadius() {
        return ((targetFieldCentricPosition.y - robotCentricCameraPosition.y) / Math.tan(cameraAngleShoot + tyShoot)) * (tvShoot ? 1 : 0);
    }

    /**
     * Using the Radius from target on a horizontal plane find the Y cord.
     * @return Y cord of the CAMERA
     */
    public double getCameraYPosition(Rotation2d gyroAngle) {
        Rotation2d invertedRobotPosition = gyroAngle.minus(new Rotation2d(270));

        return Math.cos(invertedRobotPosition.getRadians() - txShoot) * getCameraXPosition();
    }

    /**
     * Using the Radius from target on a horizontal place find the X cord
     * (ty + HalfLensView) + cameraAngle
     * @return X Cord of the CAMERA
     */
    public double getCameraXPosition() {
        return Math.sin((tyShoot + Math.toRadians(20.5)) + cameraAngleShoot) * getRobotRadius();
    }

    /**
     * Convert the two doubles into a single point {@link org.opencv.core.Point}
     * @return XY point of the camera
     */
    public Translation2d getCameraPoint(Rotation2d robotAngle) {
        return new Translation2d(getCameraXPosition(), getCameraYPosition(robotAngle));
    }

    /**
     * Translate the Camera Y into the robot Y
     * @param gyroAngle The Angle of the robot in degrees
     * @return The translated Y value of the robot
     */
    public double translateCameraYToCenter(Rotation2d gyroAngle) {
        //return Math.sin(Math.toRadians(gyroAngle)) *  Math.hypot(robotCentricCameraPosition.x, robotCentricCameraPosition.y);
        return robotCentricCameraPosition.y + getCameraYPosition(gyroAngle);
    }

    /**
     * Translate the Camera X into the robot X
     * @return The translated X value of the robot
     */
    public double translateCameraXToCenter() {
        //return Math.cos(gyroAngle) * Math.hypot(robotCentricCameraPosition.x, robotCentricCameraPosition.y);
        return robotCentricCameraPosition.x + getCameraXPosition();
    }

    /**
     * Convert the two doubles into a single point
     * @param gyroAngle Robot Angle
     * @return Center point of the robot
     */
    public Translation2d getRobotPosition(Rotation2d gyroAngle) {
        return new Translation2d(translateCameraXToCenter(), translateCameraYToCenter(gyroAngle));
    }

    public double getProcessTime() {
        return limeLightTableShoot.getEntry("tl").getDouble(11) + 11;
    }

//    public double getProbableArea(Point robotPosition) {
//        return Math.hypot(robotPosition.x - targetFieldCentricPosition.x, robotPosition.y - targetFieldCentricPosition.y) * 0;
//    }

    /**
     * Does the LimeLight have a bounding box in view
     * @return
     */
    public boolean hasTarget() {
        return tvShoot;
    }

    /**
     * Setting the LimeLight to blink the green LEDs
     */
    public void forceLEDBlink() {
        limeLightTableShoot.getEntry(LimeLightValues.LEDMODE.getTableName()).setNumber(2);
    }

    /**
     * Turn off the LimeLight LEDs
     */
    public void disableLED() {
        limeLightTableShoot.getEntry(LimeLightValues.LEDMODE.getTableName()).setNumber(1);
    }

    /**
     * Turn on the LimeLight LEDs needed to view the target
     */
    public void enableLED() {
        limeLightTableShoot.getEntry(LimeLightValues.LEDMODE.getTableName()).setNumber(3);
    }

    public double getBallRadiusToCamera() {
        return (robotCentricCameraPosition.y) / Math.tan(cameraAngleBall + tyBall);
    }

    public double getBallYToCamera() {
        return Math.sin(txBall) * getRobotRadius();
    }

    public double getBallXToCamera() {
        return Math.cos(txShoot) * getRobotRadius();
    }

    public Translation2d getBallCameraPosition() {
        return new Translation2d(getBallXToCamera(), getBallYToCamera());
    }

    public Translation2d getBallPosition(double robotAngle) {
        return new Translation2d(Math.cos(robotAngle) * cameraIntakeHypot, Math.sin(robotAngle) * cameraIntakeHypot);
    }
}