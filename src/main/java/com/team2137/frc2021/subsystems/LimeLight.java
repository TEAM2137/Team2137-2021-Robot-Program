package com.team2137.frc2021.subsystems;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.util.PID;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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

    NetworkTable limeLightTable;
    double tx, ty, ta, ledMode, camMode, pipeLine;
    private boolean tv = false;
    private double cameraAngle; //In Radians

    private Point targetFieldCentricPosition;
    private Point robotCentricCameraPosition;

    private PIDController thetaController;

    public LimeLight(double _cameraAngle, Point _robotCentricCameraPosition, Point _feildCentricTargetPosition, PID pidValues) {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        cameraAngle = Math.toRadians(_cameraAngle);
        robotCentricCameraPosition = _robotCentricCameraPosition;

        thetaController = pidValues.getWPIPIDController();

        thetaController.setSetpoint(0);
    }

    public LimeLight() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        cameraAngle = Math.toRadians(Constants.Shooter.LimeLightShootingCameraAngleDegree);
        robotCentricCameraPosition = Constants.Shooter.LimeLightShootingCameraPosition;

        thetaController = Constants.Shooter.LimeLightThetaPIDValues.getWPIPIDController();

        thetaController.setSetpoint(0);
    }

    @Override
    public void periodic() {
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
            tv = table.getEntry().getBoolean(false);
        }, 0);
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
    public Point getCameraPoint() {
        return new Point(getCameraXPosition(), getCameraYPosition());
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
    public Point getRobotPosition(double gyroAngle) {
        return new Point(translateCameraXToCenter(gyroAngle), translateCameraYToCenter(gyroAngle));
    }

    public double getProbableArea(Point robotPosition) {
        return Math.hypot(robotPosition.x - targetFieldCentricPosition.x, robotPosition.y - targetFieldCentricPosition.y) * 0;
    }

    public boolean hasTarget() {
        return tv;
    }

    public void forceLEDBlink() {

    }

    public double getRotationVector() {
        return thetaController.calculate(tx) / 12;
    }
}