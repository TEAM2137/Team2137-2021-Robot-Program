package com.team2137.frc2021.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class CommandRunner {

    private static HashMap<CommandBase, Runnable> commandBaseList = new HashMap<>();
    private static HashMap<SubsystemBase, Runnable> subsystemList = new HashMap<>();

    private static ScheduledThreadPoolExecutor threadPoolExecutor;

    private static int watchdogTimer = 25;

    public CommandRunner() {
        threadPoolExecutor = new ScheduledThreadPoolExecutor(4);
    }

    public static void executeCommand(CommandBase base) {
        Runnable tmp  = base::initialize;
        threadPoolExecutor.execute(tmp);
        threadPoolExecutor.scheduleAtFixedRate(base::execute, 0, 25, TimeUnit.MILLISECONDS);
    }

    public static void registerSubSystem(SubsystemBase subsystem) {
        Runnable tmp = () -> {
            double startTime = System.currentTimeMillis();
            subsystem.periodic();
            if(startTime - System.currentTimeMillis() > watchdogTimer)
                DriverStation.reportWarning("Loop Overrrun on " + subsystem.getName() + ": " + (startTime - System.currentTimeMillis()), true);
        };
        subsystemList.put(subsystem, tmp);
        threadPoolExecutor.scheduleAtFixedRate(tmp, 0, 25, TimeUnit.MILLISECONDS);
    }

    public static void removeSubSystem(SubsystemBase subsystemBase) {
        threadPoolExecutor.remove(commandBaseList.get(subsystemBase));
    }

    public static void purgeSubSystems() {
        threadPoolExecutor.shutdownNow();
        subsystemList.clear();
    }
}