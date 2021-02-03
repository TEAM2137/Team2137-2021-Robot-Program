package com.team2137.frc2021.program;

import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Test extends RobotContainer implements OpMode {
    @Override
    public void init() {
        drivetrain.setAllModuleRotations(Rotation2d.fromDegrees(0));
    }

    @Override
    public void periodic() {

    }

    @Override
    public void end() {
    }
}
