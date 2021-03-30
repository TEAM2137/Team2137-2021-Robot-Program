package com.team2137.frc2021.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.*;

public class BallLimeLight extends LimeLight {
    
    public BallLimeLight() {
        super(NetworkTableInstance.getDefault().getTable("limelight-2"));
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Current Path", getCurrentPath().toString());
    }

    public Path getCurrentPath() {
        var map = new HashMap<Path, Double>();

        map.put(Path.A_Red, A_Red_Point.getDistance(new Translation2d(getLimeLightValue(LimeLightValues.TX), getLimeLightValue(LimeLightValues.TY))));
        map.put(Path.A_Blue, A_Blue_Point.getDistance(new Translation2d(getLimeLightValue(LimeLightValues.TX), getLimeLightValue(LimeLightValues.TY))));
        map.put(Path.B_Red, B_Red_Point.getDistance(new Translation2d(getLimeLightValue(LimeLightValues.TX), getLimeLightValue(LimeLightValues.TY))));
        map.put(Path.B_Blue, B_Blue_Point.getDistance(new Translation2d(getLimeLightValue(LimeLightValues.TX), getLimeLightValue(LimeLightValues.TY))));

        Entry<Path, Double> min = Collections.min(map.entrySet(), new Comparator<>() {
            public int compare(Entry<Path, Double> entry1, Entry<Path, Double> entry2) {
                return entry1.getValue().compareTo(entry2.getValue());
            }
        });

        return min.getKey();
    }

    public static final Translation2d A_Red_Point = new Translation2d(1, 1);
    public static final Translation2d A_Blue_Point = new Translation2d(1, 1);
    public static final Translation2d B_Red_Point = new Translation2d(1, 1);
    public static final Translation2d B_Blue_Point = new Translation2d(1, 1);

    public enum Path {
        A_Red,
        A_Blue,
        B_Red,
        B_Blue,
        Unknown
    }
}
