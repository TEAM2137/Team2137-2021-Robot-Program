package com.team2137.frc2021.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.sql.Driver;
import java.util.HashMap;

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
    public DriverStation driverStation;
    double tx, ty, ta, tv, ledMode, camMode, pipeLine;
    private double cameraHeight;
    private double targetHeight;
    private double cameraAngle;

    public LimeLight(DriverStation station, double _cameraHeight, double _cameraAngle, double _targetHeight) {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        driverStation = station;
        cameraHeight = _cameraHeight;
        targetHeight = _targetHeight;
        cameraAngle = _cameraAngle;
    }

    @Override
    public void periodic() {
        limeLightTable.getEntry(LimeLightValues.TX.getTableName()).addListener((table) -> tx = table.getEntry().getDouble(0.0), 0);
        limeLightTable.getEntry(LimeLightValues.TY.getTableName()).addListener((table) -> ty = table.getEntry().getDouble(0.0), 0);
        limeLightTable.getEntry(LimeLightValues.TA.getTableName()).addListener((table) -> ta = table.getEntry().getDouble(0.0), 0);
        limeLightTable.getEntry(LimeLightValues.TV.getTableName()).addListener((table) -> tv = table.getEntry().getDouble(0.0), 0);
    }

    public double getRobotRadius() {
        return (targetHeight - cameraHeight) / Math.tan(cameraAngle + ty);
    }

    public double getRobotYPosition(double angleCW) {
        return 0.0;
    }
}