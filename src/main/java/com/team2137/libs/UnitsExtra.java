package com.team2137.libs;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class UnitsExtra {
    public static Pose2d metersToFeet(Pose2d pose) {
        return new Pose2d(metersToFeet(pose.getTranslation()), pose.getRotation());
    }

    public static Translation2d metersToFeet(Translation2d translation) {
        return new Translation2d(Units.metersToFeet(translation.getX()), Units.metersToFeet(translation.getY()));
    }

    public static Pose2d feetToMeters(Pose2d pose) {
        return new Pose2d(feetToMeters(pose.getTranslation()), pose.getRotation());
    }

    public static Translation2d feetToMeters(Translation2d translation) {
        return new Translation2d(Units.feetToMeters(translation.getX()), Units.feetToMeters(translation.getY()));
    }
}
