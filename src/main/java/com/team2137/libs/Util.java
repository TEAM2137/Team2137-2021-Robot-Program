package com.team2137.libs;

public class Util {

    /**
     * Clamps a value between two other values
     * @param value the value to be clamped
     * @param min the minimum allowed value
     * @param max the maximum allowed value
     * @return the clamped value
     */
    public static double clamp(double value, double min, double max) {
        if(value < min) {
            return min;
        } else if(value > max) {
            return max;
        } else {
            return value;
        }
    }

    public static double deadband(double value, double deadband) {
        if(Math.abs(value) < deadband) {
            return 0;
        } else {
            return value;
        }
    }
}
