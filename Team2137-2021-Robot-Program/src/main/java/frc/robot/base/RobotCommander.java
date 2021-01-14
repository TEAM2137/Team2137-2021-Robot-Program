/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.base;

import frc.robot.base.OpMode;
import java.util.function.Consumer;

import javax.annotation.Nonnull;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.NotifierJNI;

/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
public class RobotCommander extends RobotBase {
    
    public enum RobotStatus {
        DISABLED ("DISABLED"),
        TELEOP ("TELEOP"),
        AUTONOMOUS ("AUTONOMOUS"),
        TEST ("TEST");

        String name = "";

        RobotStatus (String _name) {
            this.name = _name;
        }

        public String toString() {
            return this.name;
        }
    }

    // public enum RobotStatus {
    //     Disabled (getDisabledSubProgram()),
    //     TeleOp (getTeleOpSubProgram()),
    //     Test (getTeleOpSubProgram()),
    //     Autonomous (getAutonomousSubProgram());

    //     OpMode linearOpMode = null;

    //     RobotStatus (OpMode opMode) {
    //         this.linearOpMode = opMode;             
    //     }

    //     public String toString() {
    //         return this.linearOpMode.getClass().getCanonicalName();
    //     }

    //     public RobotStatus ConvertRobotBase(RobotBase robo){
    //         if (robo.isOperatorControl()) return TeleOp;
    //         else if (robo.isTest()) return Test;
    //         else if (robo.isAutonomous()) return Autonomous;
    //         else return Disabled;
    //     }

    //     public void executeINIT(DriverStation ds) {
    //         reportToDS(ds, true);
    //         this.linearOpMode.init();
    //     }

    //     public void executeLOOP() {
    //         switch(this) {
    //             case Autonomous:
    //                 LiveWindow.setEnabled(false);
    //                 Shuffleboard.disableActuatorWidgets();
    //                 HAL.observeUserProgramAutonomous();
    //             case TeleOp:
    //                 LiveWindow.setEnabled(false);
    //                 Shuffleboard.disableActuatorWidgets();
    //                 HAL.observeUserProgramTeleop();
    //             case Disabled:
    //                 LiveWindow.setEnabled(false);
    //                 Shuffleboard.disableActuatorWidgets();
    //                 HAL.observeUserProgramDisabled();
    //                 break;
    //             case Test:
    //                 LiveWindow.setEnabled(true);
    //                 Shuffleboard.enableActuatorWidgets();
    //                 HAL.observeUserProgramTest();
    //                 break;
    //         }

    //         this.linearOpMode.loop();
    //     }

    //     public void executeEND(DriverStation ds) {
    //         reportToDS(ds, false);
    //         this.linearOpMode.end();
    //     }

        // public void executeSubProgram(RobotBase robo, DriverStation ds, Consumer<Boolean> afterConsumer) {
        //     reportToDS(ds, true);
        //     switch(this) {
        //         case Autonomous:
        //             LiveWindow.setEnabled(false);
        //             Shuffleboard.disableActuatorWidgets();
        //             HAL.observeUserProgramAutonomous();
        //         case TeleOp:
        //             LiveWindow.setEnabled(false);
        //             Shuffleboard.disableActuatorWidgets();
        //             HAL.observeUserProgramTeleop();
        //         case Disabled:
        //             LiveWindow.setEnabled(false);
        //             Shuffleboard.disableActuatorWidgets();
        //             HAL.observeUserProgramDisabled();
        //             break;
        //         case Test:
        //             LiveWindow.setEnabled(true);
        //             Shuffleboard.enableActuatorWidgets();
        //             HAL.observeUserProgramTest();
        //             break;
        //     }
            
        //     this.linearOpMode.init();

        //     while(this == ConvertRobotBase(robo)) {
        //         SmartDashboard.updateValues();
        //         LiveWindow.updateValues();
        //         Shuffleboard.update();
    
        //         this.linearOpMode.loop();
        //     }
    
        //     this.linearOpMode.end();

        //     reportToDS(ds, false);
        // }

    //     public void reportToDS(DriverStation ds, boolean active){
    //         switch(this){
    //             case Disabled:
    //                 ds.InDisabled(active);
    //                 break;
    //             case Autonomous:
    //                 ds.InAutonomous(active);
    //                 break;
    //             case TeleOp:
    //                 ds.InOperatorControl(active);
    //                 break;
    //             case Test:
    //                 ds.InTest(active);
    //                 break;                
    //         }
    //     }
    // }
    
    public static boolean boolRobotActive = false;
    public static boolean boolTestActive = false;

    private RobotStatus robotStatus = RobotStatus.DISABLED;
    private RobotStatus lastRobotStatus = RobotStatus.DISABLED;
    
    private static OpMode disabledSubProgram = null;
    private static OpMode autonomousSubProgram = null;
    private static OpMode teleOpSubProgram = null;

    private volatile boolean m_exit = false;

    @SuppressWarnings("PMD.CyclomaticComplexity")
    @Override
    public void startCompetition() {
        HAL.report(22, 4);
        // Tell the DS that the robot is ready to be enabled
        HAL.observeUserProgramStarting();

        while (!m_exit) {
            robotStatus = parseRobotStatus(this);
            
            if (lastRobotStatus != robotStatus) {
                switch (lastRobotStatus) {
                    case AUTONOMOUS:
                        this.autonomousSubProgram.end(false);
                        break;
                    case TELEOP:
                        this.teleOpSubProgram.end(false);
                        break;
                    case TEST:
                        this.teleOpSubProgram.end(true);
                        break;
                    case DISABLED:
                        this.disabledSubProgram.end(false);
                        break;
                }

                switch (robotStatus) {
                    case AUTONOMOUS:
                        this.autonomousSubProgram.init(false);
                        break;
                    case TELEOP:
                        this.teleOpSubProgram.init(false);
                        break;
                    case TEST:
                        this.teleOpSubProgram.init(true);
                        break;
                    case DISABLED:
                        this.disabledSubProgram.init(false);
                        break;
                }
            }

            switch (robotStatus) {
                case AUTONOMOUS:
                this.autonomousSubProgram.loop(false);
                    break;
                case TELEOP:
                    this.teleOpSubProgram.loop(false);
                    break;
                case TEST:
                    this.teleOpSubProgram.loop(true);
                    break;
                case DISABLED:
                    this.disabledSubProgram.loop(false);
                    break;
            }

            lastRobotStatus = robotStatus;
        }
    }

    @Override
    public void endCompetition() {
        m_exit = true;
    }

    public RobotStatus parseRobotStatus(RobotBase base) {
        if (base.isAutonomous()) return RobotStatus.AUTONOMOUS;
        if (base.isOperatorControl()) return RobotStatus.TELEOP;
        if (base.isTest()) return RobotStatus.TEST;
        else return RobotStatus.DISABLED;
    }

    public static OpMode getDisabledSubProgram(){
        return disabledSubProgram;
    }

    public static OpMode getAutonomousSubProgram(){
        return autonomousSubProgram;
    }

    public static OpMode getTeleOpSubProgram(){
        return teleOpSubProgram;
    }

    public static void RegisterOpModes(@Nonnull OpMode disabled, @Nonnull OpMode auton, @Nonnull OpMode tele){
        disabledSubProgram = disabled;
        autonomousSubProgram = auton;
        teleOpSubProgram = tele;
    }
}