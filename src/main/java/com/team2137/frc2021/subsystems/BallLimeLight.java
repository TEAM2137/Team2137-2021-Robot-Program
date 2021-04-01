package com.team2137.frc2021.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;
import java.util.Map.*;

public class BallLimeLight extends LimeLight {

    private final HashMap<Translation2d, Path> limelightSampleSpace = new HashMap<>();

    public BallLimeLight() {
        super(NetworkTableInstance.getDefault().getTable("limelight-2"));
        limelightSampleSpace.put(new Translation2d(17.3, -10.7), Path.A_Red);
        limelightSampleSpace.put(new Translation2d(26.0, -1.75), Path.A_Blue);
        limelightSampleSpace.put(new Translation2d(-16.3, -9.8), Path.B_Red);
        limelightSampleSpace.put(new Translation2d(16.3, -1.4), Path.B_Blue);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Current Path", getCurrentPath().toString());
    }

    //Maybe look at this change that allows for more data to be added to the sample space and it more RAM conservative and less complicated
    public Path getClosestPath() {
        Translation2d value = new Translation2d(getLimeLightValue(LimeLightValues.TX), getLimeLightValue(LimeLightValues.TY));
        return Collections.min(limelightSampleSpace.entrySet(), (o1, o2) -> {
            return (int) Math.ceil(o1.getKey().getDistance(value) - o2.getKey().getDistance(value));
        }).getValue();
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

    public static final Translation2d A_Red_Point = new Translation2d(17.3, -10.7);
    public static final Translation2d A_Blue_Point = new Translation2d(26, -1.75);
    public static final Translation2d B_Red_Point = new Translation2d(-16.3, -9.8);
    public static final Translation2d B_Blue_Point = new Translation2d(16.3, -1.4);

    public enum Path {
        A_Red,
        A_Blue,
        B_Red,
        B_Blue,
        Unknown;
    }
}
