package com.team2137.frc2021.autonomous;

import com.team2137.frc2021.RobotContainer;
import com.team2137.frc2021.subsystems.BallLimeLight.Path;
import com.team2137.frc2021.subsystems.BallLimeLight;
import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.BallLimeLight.Path;
import com.team2137.frc2021.subsystems.Spindexer;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Map;

public class GalacticSearch extends SequentialCommandGroup {

    public GalacticSearch(SwerveDrivetrain drivetrain, Intake intake, Spindexer spindexer, BallLimeLight limelight) {
        var command = new SelectCommand(
                Map.ofEntries(
                        Map.entry(Path.A_Red, new GalacticSearchARed(drivetrain, intake, spindexer)),
                        Map.entry(Path.A_Blue, new GalacticSearchABlue(drivetrain, intake, spindexer)),
                        Map.entry(Path.B_Red, new GalacticSearchBRed(drivetrain, intake, spindexer)),
                        Map.entry(Path.B_Blue, new GalacticSearchBBlue(drivetrain, intake, spindexer))
                ),
                limelight::getClosestPath
        );

        addCommands(command);
    }
}
