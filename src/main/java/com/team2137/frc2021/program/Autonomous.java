package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.autonomous.BarrelRacing;
import com.team2137.frc2021.autonomous.BouncePath;
import com.team2137.frc2021.autonomous.GalacticSearch;
import com.team2137.frc2021.autonomous.SlalomPath;
import edu.wpi.first.wpilibj.DriverStation;

public class Autonomous extends RobotContainer implements OpMode {

    public enum Challenge {
        BARRELRACING ("Barrel Racing"),
        BOUNCEPATH ("Bounce Path"),
        GALACTICSEARCH ("Galactic Search"),
        SLALOMPATH ("Slalom Path"),
        UNKNOWN ("Unknown");

        private String stringValue = "";

        Challenge (String name) {
            stringValue = name;
        }
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void end() {

    }
}
