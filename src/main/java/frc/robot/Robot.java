package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.base.LinearOpMode;
import frc.robot.base.OpMode;

import javax.annotation.Nonnull;

/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
public class Robot extends RobotBase {
    private static LinearOpMode autonomousSubProgram;
    private static LinearOpMode teleOpSubProgram;
    private static LinearOpMode disabledSubProgram;

    private volatile boolean m_exit;

    public enum RobotStatus {
        DISABLED, AUTONOMOUS, TELEOP, TEST, SIMULATION
    }

    public RobotStatus parseRobotStatus(RobotBase base) {
        if (base.isDisabled())
            return RobotStatus.DISABLED;
        else if (base.isAutonomous())
            return RobotStatus.AUTONOMOUS;
        else if (base.isOperatorControl())
            return RobotStatus.TELEOP;
        else if (base.isTest())
            return RobotStatus.TEST;
        else
            return RobotStatus.DISABLED;
    }

    private void waitForStatusChange(RobotStatus startStatus, LinearOpMode opmode) {
        while (startStatus == parseRobotStatus(this) && m_exit == false);
        opmode.EndOpMode();
    }

    @SuppressWarnings("PMD.CyclomaticComplexity")
    @Override
    public void startCompetition() {
        // robotInit();

        // Tell the DS that the robot is ready to be enabled
        HAL.observeUserProgramStarting();

        while (!m_exit) {
            switch (parseRobotStatus(this)) {
                case DISABLED:
                    m_ds.InDisabled(true);
                    System.out.println("INIT Disabled");
                    try {
                        disabledSubProgram = disabledSubProgram.getClass().newInstance();
                    } catch (Exception e) {
                        System.out.println("Failed to create the disabled SubProgram");
                        e.printStackTrace();
                    }
                        disabledSubProgram.start();
                        waitForStatusChange(RobotStatus.DISABLED, disabledSubProgram);
                        m_ds.InDisabled(false);
                        break;
                    case AUTONOMOUS:
                        System.out.println("INIT Autonomous");
                        m_ds.InAutonomous(true);
                        try {
                            autonomousSubProgram = autonomousSubProgram.getClass().newInstance();
                        } catch (Exception e) {
                            System.out.println("Failed to create the disabled SubProgram");
                            e.printStackTrace();
                        }
                        autonomousSubProgram.start();
                        waitForStatusChange(RobotStatus.AUTONOMOUS, autonomousSubProgram);
                        m_ds.InAutonomous(false);
                        break;
                    case TELEOP:
                        System.out.println("INIT TeleOp");
                        m_ds.InOperatorControl(true);
                        try {
                            teleOpSubProgram = teleOpSubProgram.getClass().newInstance();
                        } catch (Exception e) {
                            System.out.println("Failed to create the disabled SubProgram");
                            e.printStackTrace();
                        }
                        teleOpSubProgram.start();
                        waitForStatusChange(RobotStatus.TELEOP, teleOpSubProgram);
                        m_ds.InOperatorControl(false);
                        break;
                    case TEST:
                        System.out.println("INIT Test");
                        LiveWindow.setEnabled(true);
                        Shuffleboard.enableActuatorWidgets();
                        m_ds.InTest(true);
                        try {
                            teleOpSubProgram = teleOpSubProgram.getClass().newInstance();
                        } catch (Exception e) {
                            System.out.println("Failed to create the disabled SubProgram");
                            e.printStackTrace();
                        }

                        teleOpSubProgram.start();
                        waitForStatusChange(RobotStatus.TEST, teleOpSubProgram);
                        m_ds.InTest(false);
                        LiveWindow.setEnabled(false);
                        Shuffleboard.disableActuatorWidgets();
                        break;
            }
        }
    }

    @Override
    public void endCompetition() {
        System.out.println("End Competition Called");
        m_exit = true;
    }

    public static void registerOpModes(@Nonnull LinearOpMode autonomous, @Nonnull LinearOpMode teleOp, @Nonnull LinearOpMode disabled) {
        autonomousSubProgram = autonomous;
        teleOpSubProgram = teleOp;
        disabledSubProgram = disabled;
    }
}
