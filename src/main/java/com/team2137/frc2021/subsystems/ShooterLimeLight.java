package com.team2137.frc2021.subsystems;

import com.team2137.frc2021.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterLimeLight extends LimeLight {

    public ShooterLimeLight() {
        super(NetworkTableInstance.getDefault().getTable("limelight"));
    }

    public double getRadialDistance() {
        return (Constants.Shooter.LimeLightTargetFieldPosition.y - Constants.Shooter.LimeLightShootingCameraPosition.y) / Math.tan(Math.toRadians(Constants.Shooter.LimeLightShootingCameraAngleDegree) + getLimeLightValue(LimeLightValues.TY));
    }

    @Override
    public void periodic() {

    }
}
