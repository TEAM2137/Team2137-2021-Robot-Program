package com.team2137.frc2021.program;

import com.team2137.libs.InterpolationMap;

public class ShooterMap {
    private static InterpolationMap flywheelMap = new InterpolationMap();
    private static InterpolationMap hoodMap = new InterpolationMap();
    //queue sadness
    static {
        flywheelMap.put(5.0, 3000.0); // temp values
        hoodMap.put(5.0, 0.0);
    }

    public static double getFlywheelSpeed(double distanceMeters) {
        return flywheelMap.get(distanceMeters);
    }

    public static double getHoodAngle(double distanceMeters) {
        return hoodMap.get(distanceMeters);
    }
}
