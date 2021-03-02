package com.team2137.frc2021.subsystems;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.util.PID;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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

    private NetworkTable limeLightTable; //Network table for getting the LimeLight values
    /**
     * tx - the degrees the target is on the camera
     * ty - the degrees the target is vertically on the camera
     * ta - the area of the bounding box in the total field of view
     */
    private double tx, ty, ta; // Target position values
    private boolean tv = false;
    private double cameraAngle; //In Radians

    private Point targetFieldCentricPosition;
    private Point robotCentricCameraPosition;

    public LimeLight(double _cameraAngle, Point _robotCentricCameraPosition, Point _feildCentricTargetPosition, PID pidValues) {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        cameraAngle = Math.toRadians(_cameraAngle);
        robotCentricCameraPosition = _robotCentricCameraPosition;
    }

    /**
     * Create a new LimeLight object for the Subsystem using the default values in {@link com.team2137.frc2021.Constants}
     */
    public LimeLight() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        cameraAngle = Math.toRadians(Constants.Shooter.LimeLightShootingCameraAngleDegree);
        robotCentricCameraPosition = Constants.Shooter.LimeLightShootingCameraPosition;

        targetFieldCentricPosition = Constants.Shooter.LimeLightTargetFieldPosition;

        limeLightTable.getEntry(LimeLightValues.TX.getTableName()).addListener((table) -> {
            tx = Math.toRadians(table.getEntry().getDouble(0.0));
        }, 0);
        limeLightTable.getEntry(LimeLightValues.TY.getTableName()).addListener((table) -> {
            ty = Math.toRadians(table.getEntry().getDouble(0.0));
        }, 0);
        limeLightTable.getEntry(LimeLightValues.TA.getTableName()).addListener((table) -> {
            ta = table.getEntry().getDouble(0.0);
        }, 0);
        limeLightTable.getEntry(LimeLightValues.TV.getTableName()).addListener((table) -> {
            tv = table.getEntry().getDouble(0) > 1.0;
        }, 0);
    }

    @Override
    public void periodic() {

    }

    /**
     * Find the radius of the arc of the robot to the target or the distance to the target
     * @return Camera distance from object on horizontal plane
     */
    public double getRobotRadius() {
        return (targetFieldCentricPosition.y - robotCentricCameraPosition.y) / Math.tan(cameraAngle + ty);
    }

    /**
     * Using the Radius from target on a horizontal plane find the Y cord.
     * @return Y cord of the CAMERA
     */
    public double getCameraYPosition() {
        return Math.sin(tx) * getRobotRadius();
    }

    /**
     * Using the Radius from target on a horizontal place find the X cord
     * @return X Cord of the CAMERA
     */
    public double getCameraXPosition() {
        return Math.cos(tx) * getRobotRadius();
    }

    /**
     * Convert the two doubles into a single point {@link org.opencv.core.Point}
     * @return XY point of the camera
     */
    public Translation2d getCameraPoint() {
        return new Translation2d(getCameraXPosition(), getCameraYPosition());
    }

    /**
     * Translate the Camera Y into the robot Y
     * @param gyroAngle The Angle of the robot in degrees
     * @return The translated Y value of the robot
     */
    public double translateCameraYToCenter(double gyroAngle) {
        return Math.sin(Math.toRadians(gyroAngle)) *  Math.hypot(robotCentricCameraPosition.x, robotCentricCameraPosition.y);
    }

    /**
     * Translate the Camera X into the robot X
     * @param gyroAngle The Angle of the robot in degrees
     * @return The translated X value of the robot
     */
    public double translateCameraXToCenter(double gyroAngle) {
        return Math.cos(gyroAngle) * Math.hypot(robotCentricCameraPosition.x, robotCentricCameraPosition.y);
    }

    /**
     * Convert the two doubles into a single point {@link org.opencv.core.Point}
     * @param gyroAngle Robot Angle
     * @return Center point of the robot
     */
    public Translation2d getRobotPosition(double gyroAngle) {
        return new Translation2d(translateCameraXToCenter(gyroAngle), translateCameraYToCenter(gyroAngle));
    }

//    public double getProbableArea(Point robotPosition) {
//        return Math.hypot(robotPosition.x - targetFieldCentricPosition.x, robotPosition.y - targetFieldCentricPosition.y) * 0;
//    }

    /**
     * Does the LimeLight have a bounding box in view
     * @return
     */
    public boolean hasTarget() {
        return tv;
    }

    /**
     * Setting the LimeLight to blink the green LEDs
     */
    public void forceLEDBlink() {
        limeLightTable.getEntry(LimeLightValues.LEDMODE.getTableName()).setNumber(2);
    }

    /**
     * Turn off the LimeLight LEDs
     */
    public void disableLED() {
        limeLightTable.getEntry(LimeLightValues.LEDMODE.getTableName()).setNumber(1);
    }

    /**
     * Turn on the LimeLight LEDs needed to view the target
     */
    public void enableLED() {
        limeLightTable.getEntry(LimeLightValues.LEDMODE.getTableName()).setNumber(3);
    }
}