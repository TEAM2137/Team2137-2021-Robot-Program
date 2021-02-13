package com.team2137.frc2021;

import com.team2137.frc2021.util.PID;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    public static class Drivetrain {
        public static final double length = Units.inchesToMeters(24.5);
        public static final double width = Units.inchesToMeters(21.5);

        public static final double motorToWheelConversionFactor = (1 / 6.86) * Units.inchesToMeters(4 * Math.PI);

        public static final double driveMaxSpeed = Units.feetToMeters(13.5); // temp value
        public static final double driveMaxAccel = Units.feetToMeters(8.0); // temp value

        public static final boolean invertDriveMotor = true;
        public static final boolean invertTurningMotor = false;
        public static final int driveMotorCurrentLimit = 80;
        public static final int turningMotorCurrentLimit = 80;

        public static final double driveMotorRamp = 0.5;

        public static PID turningPIDConstants = new PID(0.075, 0, -0.000000000000000000001); // in the air
//        public staticPID turningPIDConstants = new PID(0.1, 0, -0.0000000000000000000000001); // carpet
//        public static PID turningPIDConstants = new PID(0.08, 0, 0); // carpet

        public static PID drivePIDConstants = new PID(0.32, 0, 0);
        public static SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.17, 2.7, 0);

        public static SwerveModuleConstants frontLeft = new SwerveModuleConstants(10, 11, 12, -55.81, "Front Left");
        public static SwerveModuleConstants frontRight = new SwerveModuleConstants(15, 16, 17, 120.94, "Front Right");
        public static SwerveModuleConstants backLeft = new SwerveModuleConstants(20, 21, 22, 179.56, "Back Left");
        public static SwerveModuleConstants backRight = new SwerveModuleConstants(25, 26, 27, 9.59, "Back Right");

        public static PID translationPIDConstants = new PID(0, 0, 0);
        public static PID thetaPIDConstants = new PID(0, 0, 0);
        public static TrapezoidProfile.Constraints thetaPIDConstraints = new TrapezoidProfile.Constraints(2, 1);

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
        STATE_FINISH ("STATE_FINISHED");

        private String name = "";

        StepState(String name) {
            this.name = name;
        }

        public String toString() {
            return this.name;
        }
    }
}
