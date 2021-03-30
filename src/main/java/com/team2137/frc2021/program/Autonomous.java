package com.team2137.frc2021.program;

import com.team2137.frc2021.Constants;
import com.team2137.frc2021.OpMode;
import com.team2137.frc2021.RobotContainer;

public class Autonomous extends RobotContainer implements OpMode {

    public enum Challenge {
        BarrelRacing ("Barrel Racing"),
        BouncePath ("Bounce Path"),
        GalacticSearch ("Galactic Search"),
        SlalomPath ("Slalom Path"),
        Unknown ("Unknown");

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
