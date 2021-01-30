package com.team2137.frc2021.program;

import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Autonomous extends RobotContainer implements OpMode {
    Timer stopTimer = new Timer();

    @Override
    public void init() {
        drivetrain.setAllModuleRotations(Rotation2d.fromDegrees(0));
        drivetrain.setAllModuleDriveVelocity(Units.feetToMeters(8));
    }

    @Override
    public void periodic() {
        if(stopTimer.hasElapsed(3)) {
            drivetrain.setAllModuleDriveVelocity(0);
        }
    }

    @Override
    public void end() {
        drivetrain.setAllModuleDriveVelocity(0);
        drivetrain.setAllModuleDriveRawPower(0);
    }
}
