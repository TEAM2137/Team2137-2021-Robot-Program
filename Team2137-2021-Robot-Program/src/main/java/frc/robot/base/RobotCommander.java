/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.base;

import java.util.function.Consumer;

import javax.annotation.Nonnull;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
public class RobotCommander extends RobotBase {
    
    public enum RobotStatus {
        Disabled (getDisabledSubProgram()),
        TeleOp (getTeleOpSubProgram()),
        Test (getTeleOpSubProgram()),
        Autonomous (getAutonomousSubProgram());

        LinearOpMode linearOpMode = null;

        RobotStatus (LinearOpMode opMode) {
            this.linearOpMode = opMode;             
        }

        public RobotStatus ConvertRobotBase(RobotBase robo){
            if (robo.isOperatorControl()) return TeleOp;
            else if (robo.isTest()) return Test;
            else if (robo.isAutonomous()) return Autonomous;
            else return Disabled;
        }

        public void executeSubProgram(RobotBase robo, DriverStation ds, Consumer<Boolean> afterConsumer) {
            reportToDS(ds, true);
            this.linearOpMode.run(this != Test);
            reportToDS(ds, false);

            while (ConvertRobotBase(robo) == this) afterConsumer.accept(true);
        }

        public void reportToDS(DriverStation ds, boolean active){
            switch(this){
                case Disabled:
                    ds.InDisabled(active);
                    break;
                case Autonomous:
                    ds.InAutonomous(active);
                    break;
                case TeleOp:
                    ds.InOperatorControl(active);
                    break;
                case Test:
                    ds.InTest(active);
                    break;                
            }
        }
    }
    
    public static boolean boolRobotActive = false;
    public static boolean boolTestActive = false;

    private RobotStatus robotStatus = RobotStatus.Disabled;
    
    private static LinearOpMode disabledSubProgram = null;
    private static LinearOpMode autonomousSubProgram = null;
    private static LinearOpMode teleOpSubProgram = null;

    private volatile boolean m_exit;

    @SuppressWarnings("PMD.CyclomaticComplexity")
    @Override
    public void startCompetition() {
        // Tell the DS that the robot is ready to be enabled
        HAL.observeUserProgramStarting();

        while (!Thread.currentThread().isInterrupted() && !m_exit) {
            robotStatus = RobotStatus.Disabled.ConvertRobotBase(this);
            robotStatus.executeSubProgram(this, m_ds, e -> {
                m_ds.waitForData();
            });
        }
    }

    @Override
    public void endCompetition() {
        m_exit = true;
    }

    public static LinearOpMode getDisabledSubProgram(){
        return disabledSubProgram;
    }

    public static LinearOpMode getAutonomousSubProgram(){
        return autonomousSubProgram;
    }

    public static LinearOpMode getTeleOpSubProgram(){
        return teleOpSubProgram;
    }

    public static void RegisterOpModes(@Nonnull LinearOpMode disabled, @Nonnull LinearOpMode auton, @Nonnull LinearOpMode tele){
        disabledSubProgram = disabled;
        autonomousSubProgram = auton;
        teleOpSubProgram = tele;
    }
}