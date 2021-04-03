package com.team2137.frc2021;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team2137.frc2021.util.Motor;
import com.team2137.frc2021.util.PID;
import com.team2137.libs.GamePad;
import com.ctre.phoenix.CANifier.GeneralPin;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import org.opencv.core.Point;

public class Constants {

    public enum ShooterPresets {
        Off(0, 0, 0),

        InitiationLine (0, 5000, .5),
        Tench (0, 6000, .5),

        AutoShoot(17, 3800, 1)
        ;

        public double hoodAngle = 0;
        public int flywheelSpeed = 0;
        public double prerollerPower = 0;

        ShooterPresets(double _hoodAngle, int _flywheelSpeed, double _prerollerPower) {
            hoodAngle = _hoodAngle;
            flywheelSpeed = _flywheelSpeed;
            prerollerPower = _prerollerPower;
        }
    }

    public static class Shooter {
        //Parm 0 - Ramp Speed ~ Parm 1 - P ~ Parm 2 - I ~ Parm 3 - D ~ Parm 4 - Max Vel
//        public static final Motor FlyWheelMotorObject1 = new Motor("flyWheelMotor1", 42, Motor.MotorTypes.FALCON, false, 60, 1, 5.0, new PID(0.10, 0.00, 0.00, 0.4, 0.106, 0.0225), "6000"); //Parm 4 - Max Vel
        public static final Motor FlyWheelMotorObject1 = new Motor("flyWheelMotor1", 42, Motor.MotorTypes.FALCON, false, 60, 1, 2, new PID(5.0, 0.0, 0.55, 0.4, 0.106, 0.0225), "6000"); //Parm 4 - Max Vel
        public static final Motor FlyWheelMotorObject2 = new Motor("flyWheelMotor2", 43, Motor.MotorTypes.FALCON, true, 60, 1, 2, null);
        public static final Motor PreRollerMotorObject = new Motor("preRollerMotor", 44, Motor.MotorTypes.NEO550, true, 35, 1, 0, null);
        public static final Motor HoodMotorObject = new Motor("hoodMotor", 41, Motor.MotorTypes.NEO550, false, 35, ((1.0/90.0) * (32.0/70.0)) * 360.0, 0, new PID(0.03, 0, 0), "45", "0"); //Parm 4 - Max Position deg ~ Parm 5 Min Position deg  TODO fix gear ratio for hood motor (Rotation Per Degree)

        public static final Point LimeLightShootingCameraPosition = new Point(0, 1.5);
        public static final Point LimeLightTargetFieldPosition = new Point(0, 7.5);
        public static final double LimeLightShootingCameraAngleDegree = 32.47;
        public static final double FlywheelIdlePercent = 0.0; //0.5;
        public static final int HoodMotorHomingCurrentSignal = 20;
    }

    public static class Drivetrain {
        public static final int gyroID = 4;

        public static final double length = Units.inchesToMeters(24.5);
        public static final double width = Units.inchesToMeters(21.5);

        public static final double motorToWheelConversionFactor = (1 / 6.86) * Units.inchesToMeters(4 * Math.PI);

        public static final double driveMaxSpeed = Units.feetToMeters(13); // temp value
        public static final double driveMaxAccel = Units.feetToMeters(6.0); // temp value

        public static final boolean invertDriveMotor = true;
        public static final boolean invertTurningMotor = false;
        public static final SupplyCurrentLimitConfiguration driveMotorSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 60 ,1);
        public static final StatorCurrentLimitConfiguration driveMotorStatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 40, 60, 1);
        public static final SupplyCurrentLimitConfiguration turningMotorSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 1);
        public static final StatorCurrentLimitConfiguration turningMotorStatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 20, 30, 1);

        public static final double driveMotorRamp = 0.5;

        public static double turningFeedForward = 0.75; //0.8
//        public static PID turningPIDConstants = new PID(0.21, 0, 0.0015); // in the air
//        public staticPID turningPIDConstants = new PID(0.1, 0, -0.0000000000000000000000001); // carpet
        public static PID turningPIDConstants = new PID(0.03, 0, 0.000883); // carpet

        public static PID drivePIDConstants = new PID(0.07, 0, 0);// 0.1
        public static SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.5, 2.15, 0); //0.7, 2.15

        public static SwerveModuleConstants frontLeft = new SwerveModuleConstants(10, 11, 12, -173.58, "Front Left");
        public static SwerveModuleConstants frontRight = new SwerveModuleConstants(15, 16, 17, 80.51, "Front Right");
        public static SwerveModuleConstants backLeft = new SwerveModuleConstants(20, 21, 22, -52.99, "Back Left");
        public static SwerveModuleConstants backRight = new SwerveModuleConstants(25, 26, 27, 121.11, "Back Right");

        public static PID translationPIDConstants = new PID(.05, 0, 0);

//        public static PID teleopThetaPIDConstants = new PID(2.3, 0, 0.1);
        public static PID teleopThetaPIDConstants = new PID(0.9, 0.0, 0.4);
        public static TrapezoidProfile.Constraints teleopThetaPIDConstraints = new TrapezoidProfile.Constraints(6, 4); // new

        public static PID autoThetaPIDConstants = new PID(2.75, 0, 0);
        public static TrapezoidProfile.Constraints autoThetaPIDConstraints = new TrapezoidProfile.Constraints(16, 16); // old

        public static PID purePIDTranslationConstants = new PID(0, 0, 0);

        public static class SwerveModuleConstants {
            public final int driveID;
            public final int turningID;
            public final int encoderID;

            public final double offset;
            public final String moduleName;

            SwerveModuleConstants(int driveID, int turningID, int encoderID, double offsetDegrees, String moduleName) {
                this.driveID = driveID;
                this.turningID = turningID;
                this.encoderID = encoderID;
                this.offset = offsetDegrees;
                this.moduleName = moduleName;
            }
        }
    }

    public enum StepState {
        STATE_INIT ("STATE_INIT"),
        STATE_RUNNING ("STATE_RUNNING"),
        STATE_FINISH ("STATE_FINISHED"),
        STATE_NOT_STARTED ("STATE_NOT_STARTED");

        private String name = "";

        StepState(String name) {
            this.name = name;
        }

        public boolean isFinished() {
            return this == STATE_FINISH;
        }

        public String toString() {
            return this.name;
        }
    }

    public static class Intake {
        public static int motorID = 30;
        public static final boolean invertMotor = false;
        public static int currentLimit = 20;

        public static final int cylinderForwardID = 2;
        public static final int cylinderReverseID = 1;
        public static final int cylinder2ForwardID = 0;
        public static final int cylinder2ReverseID = 5;
    }

    public static class Spindexer {
        public static int motorID = 35;
        public static int solenoidForwardID = 3;
        public static int solenoidReverseID = 4;

        public static boolean invertMotor = false;

//        public static int currentLimit = 30;
//        public static double voltageRamp = 0.25;
    }

    public static int canifierID = 9;

    public static boolean withinClip(double num, double goal, double width) {
        return Math.abs(goal - num) <= width;
    }

    public static double squareWithSign(double number) {
        return Math.pow(number, 2) * (number < 0 ? -1 : 1);
    }
}
