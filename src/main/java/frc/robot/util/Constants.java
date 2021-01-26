package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class Constants {

    public static double fromHeadingTo360(double a) {
        return a < 0 ? a + 360 : a;
    }

    /**
     * Turns the controller input into field centric values
     * @param xSpeed - Desired X speed on field
     * @param ySpeed - Desired Y speed on field
     * @param radianPerSecond - Desired Turn speed on field
     * @param robotAngle - Current robot Angle on field
     * @return Array or double {xSpeed, ySpeed, turnSpeed}
     */
    public static double[] toFieldRelativeChassisSpeeds(double xSpeed, double ySpeed, double radianPerSecond, Rotation2d robotAngle) {
        return new double[]{
                xSpeed * robotAngle.getCos() + ySpeed * robotAngle.getSin(),
                -xSpeed * robotAngle.getSin() + ySpeed * robotAngle.getCos(),
                radianPerSecond};
    }

    public static double getAverage(double[] values) {
        return Arrays.stream(values).sum() / values.length;
    }

    public static double[] getAverage(List<Map.Entry<Double, Double>> values) {
        double returner1 = 0;
        double returner2 = 0;
        for (Map.Entry<Double, Double> i : values) {
            returner1 += i.getKey();
            returner2 += i.getValue();
        }

        return new double[] {returner1 / values.size(), returner2 / values.size()};
    }
}
