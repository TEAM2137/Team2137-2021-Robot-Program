package com.team2137.libs;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtility {
    /**
     * @param startPose start pose of the trajectory in feet
     * @param interiorWaypoints interior waypoints of the trajectory in feet
     * @param endPose end pose of the trajectory in feet
     * @param config TrajectoryConfig in meters - will NOT be converted to feet
     * @return
     */
    public static Trajectory generateTrajectoryFeet(Pose2d startPose, List<Translation2d> interiorWaypoints, Pose2d endPose, TrajectoryConfig config) {
        Pose2d startMeters = UnitsExtra.feetToMeters(startPose);
        List<Translation2d> interiorMeters = new ArrayList<>();
        for(var waypoint : interiorWaypoints) {
            interiorMeters.add(UnitsExtra.feetToMeters(waypoint));
        }
        Pose2d endMeters = UnitsExtra.feetToMeters(endPose);
        return TrajectoryGenerator.generateTrajectory(startMeters, interiorMeters, endMeters, config);
    }
}
