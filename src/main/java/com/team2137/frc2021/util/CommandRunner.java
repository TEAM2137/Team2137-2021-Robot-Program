package com.team2137.frc2021.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.*;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.function.BiConsumer;

public class CommandRunner {

    private static class ConcurrentCommandBundle {
        private Object command;
        private Runnable runnable;
        private boolean initFlag;
        private boolean endFlag;

        public ConcurrentCommandBundle(CommandBase _base) {
            command = _base;
            initFlag = false;
            endFlag = false;
        }

        public ConcurrentCommandBundle(SubsystemBase _base) {
            command = _base;
            initFlag = false;
            endFlag = false;
        }

        public void flagInit() {
            initFlag = true;
        }

        public void flagEnd() {
            endFlag = true;
        }

        public boolean hasInitStarted() {
            return initFlag;
        }

        public boolean hasEndStarted() {
            return endFlag;
        }

        public void setRunnable(Runnable run) {
            runnable = run;
        }

        public Runnable getRunnable() {
            return runnable;
        }

        public CommandBase getCommand() {
            return (CommandBase) command;
        }

        public SubsystemBase getSubsystem() {
            return (SubsystemBase) command;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            ConcurrentCommandBundle that = (ConcurrentCommandBundle) o;
            return Objects.equals(command, that.command);
        }

        @Override
        public int hashCode() {
            return Objects.hash(command);
        }
    }

    private static List<ConcurrentCommandBundle> commandBundleList = new ArrayList<>();
    private static List<ConcurrentCommandBundle> subsystemBundleList = new ArrayList<>();

    private static ScheduledThreadPoolExecutor threadPoolExecutor;

    private static int watchdogTimer = 25;

    public CommandRunner() {
        threadPoolExecutor = new ScheduledThreadPoolExecutor(4);
    }

    public static void executeCommandGroup(CommandBase... base) {
        for(CommandBase a : base)
            executeCommand(a);
    }

    public static void executeCommand(CommandBase base) {
        ConcurrentCommandBundle bundle = new ConcurrentCommandBundle(base);
        commandBundleList.add(bundle);

        bundle.setRunnable(() -> {
            double start = System.currentTimeMillis();
            if (!bundle.hasInitStarted()) {
                bundle.getCommand().initialize();
                bundle.flagInit();
            }

            if (bundle.getCommand().isFinished()) {
                threadPoolExecutor.remove(bundle.getRunnable());
                commandBundleList.remove(bundle);
                bundle.getCommand().end(false);
                if (!bundle.hasEndStarted()) {
                    bundle.getCommand().end(false);
                    bundle.flagEnd();
                }
            } else {
                bundle.getCommand().execute();
            }

            if (System.currentTimeMillis() - start > watchdogTimer)
                DriverStation.reportWarning("Loop Overrrun on " + base.getName() + ": " + (start - System.currentTimeMillis()), true);
        });

        threadPoolExecutor.scheduleAtFixedRate(bundle.getRunnable(), 0, 25, TimeUnit.MILLISECONDS);
    }

    private static void executeCommandSequence(int index, CommandBase... commands) {
        ConcurrentCommandBundle bundle = new ConcurrentCommandBundle(commands[index]);
        commandBundleList.add(bundle);

        bundle.setRunnable(() -> {
            double start = System.currentTimeMillis();
            if(!bundle.hasInitStarted()) {
                bundle.getCommand().initialize();
                bundle.flagInit();
            }

            bundle.getCommand().execute();

            if (commands[index].isFinished()) {
                threadPoolExecutor.remove(bundle.getRunnable());
                commandBundleList.remove(bundle);

                if(commands.length > index + 1) {
                    executeCommand(commands[index + 1]);
                }
            }

            if (System.currentTimeMillis() - start > watchdogTimer)
                DriverStation.reportWarning("Loop Overrrun on " + commands[index].getName() + ": " + (start - System.currentTimeMillis()), true);
        });

        threadPoolExecutor.scheduleAtFixedRate(bundle.getRunnable(), 0, 25, TimeUnit.MILLISECONDS);
    }

    public static void executeCommandSequence(CommandBase... commands) {
        executeCommandSequence(0, commands);
    }

    public static void registerSubSystem(SubsystemBase subsystem) {
        ConcurrentCommandBundle bundle = new ConcurrentCommandBundle(subsystem);
        subsystemBundleList.add(bundle);

        bundle.setRunnable(() -> {
            double startTime = System.currentTimeMillis();
            subsystem.periodic();
            if (startTime - System.currentTimeMillis() > watchdogTimer)
                DriverStation.reportWarning("Loop Overrrun on " + subsystem.getName() + ": " + (startTime - System.currentTimeMillis()), true);
        });

        threadPoolExecutor.scheduleAtFixedRate(bundle.getRunnable(), 0, 25, TimeUnit.MILLISECONDS);
    }

    public static void registerSubSystem(SubsystemBase... subsystemBases) {
        for (SubsystemBase a : subsystemBases)
            registerSubSystem(a);
    }

    public static void purgeSubSystems() {
        subsystemBundleList.forEach((commandBundle -> threadPoolExecutor.remove(commandBundle.getRunnable())));
        subsystemBundleList.clear();
    }

    public static void purgeCommands() {
        commandBundleList.forEach((commandBundle -> threadPoolExecutor.remove(commandBundle.getRunnable())));
        commandBundleList.clear();
    }

    public static void shutdownNow() {
        threadPoolExecutor.shutdown();
    }

    public static void ReportCurrentCommands() {
        System.out.println(System.currentTimeMillis() + ": Current Commands Registered...");
        commandBundleList.forEach((commandBundle) -> System.out.println(commandBundle.getCommand().toString()));
    }

    public static void ReportCurrentRegisteredSubSystems() {
        System.out.println(System.currentTimeMillis() + ": Current SubSystems Registered...");
        subsystemBundleList.forEach((commandBundle) -> System.out.println(commandBundle.getCommand().toString()));
    }
}