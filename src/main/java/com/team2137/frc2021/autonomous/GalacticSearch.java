package com.team2137.frc2021.autonomous;

import com.team2137.frc2021.subsystems.Intake;
import com.team2137.frc2021.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Map;

public class GalacticSearch extends SequentialCommandGroup {

    public GalacticSearch(SwerveDrivetrain drivetrain, Intake intake) {
        var command = new SelectCommand(
                Map.ofEntries(
                        Map.entry()
                )
        )
    }
}
