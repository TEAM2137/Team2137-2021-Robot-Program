package com.team2137.frc2021.autonomous;

import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.subsystems.BallLimeLight.Path;

public class GalacticSearch {
    private Path mCurrentAutonPath = Path.Unknown;
    public GalacticSearch(RobotContainer robot) {
        mCurrentAutonPath = robot.ballLimelight.getCurrentPath();

        switch (mCurrentAutonPath) {
            case A_Red:
                new GalacticSearchARed(robot);
                break;
            case B_Red:
                new GalacticSearchBRed(robot);
                break;
            case A_Blue:
                new GalacticSearchABlue(robot);
                break;
            case B_Blue:
                new GalacticSearchBBlue(robot);
                break;
            default:
                break;
        }
    }
}
