package com.team2137.frc2021;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {
    public static class Drivetrain {
        public static final double length = Units.inchesToMeters(24.5);
        public static final double width = Units.inchesToMeters(21.5);

        public static final double motorToWheelConversionFactor = (1 / 6.86) * Units.inchesToMeters(4 * Math.PI);

        public static final double driveMaxSpeed = Units.feetToMeters(13.5); // temp value
        public static final double driveMaxAccel = Units.feetToMeters(4.0); // temp value

        public static final boolean invertDriveMotor = true;
        public static final boolean invertTurningMotor = false;
        public static final int driveMotorCurrentLimit = 80;
        public static final int turningMotorCurrentLimit = 80;

        public static final double driveMotorRamp = 0.5;

        public static PIDConstants turningPIDConstants = new PIDConstants(0.075, 0, .001); // in the air
//        public static PIDConstants turningPIDConstants = new PIDConstants(0.1, 0, -0.0000000000000000000000001); // carpet
//        public static PIDConstants turningPIDConstants = new PIDConstants(0.08, 0, 0); // carpet

        public static PIDConstants drivePIDConstants = new PIDConstants(0.32, 0, 0);
        public static SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.17, 2.7, 0);

        public static SwerveModuleConstants frontLeft = new SwerveModuleConstants(10, 11, 12, -55.81, "Front Left");
        public static SwerveModuleConstants frontRight = new SwerveModuleConstants(15, 16, 17, 120.94, "Front Right");
        public static SwerveModuleConstants backLeft = new SwerveModuleConstants(20, 21, 22, 179.56, "Back Left");
        public static SwerveModuleConstants backRight = new SwerveModuleConstants(25, 26, 27, 9.59, "Back Right");

        public static PIDConstants translationPIDConstants = new PIDConstants(.02, 0, 0.25);
        public static PIDConstants teleopThetaPIDConstants = new PIDConstants(0.8, 0, 5); // new
        public static TrapezoidProfile.Constraints teleopThetaPIDConstraints = new TrapezoidProfile.Constraints(6, 4); // new
        public static PIDConstants autoThetaPIDConstants = new PIDConstants(2, 0, 0); // old
        public static TrapezoidProfile.Constraints autoThetaPIDConstraints = new TrapezoidProfile.Constraints(8, 8); // old

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

    public static class PIDConstants {
        public final double P;
        public final double I;
        public final double D;

        PIDConstants(double p, double i, double d) {
            this.P = p;
            this.I = i;
            this.D = d;
        }
    }

    public static class Intake {
        public static int motorID = 30;
        
        public static int cylinderForwardID = 0;
        public static int cylinderReverseID = 1;
    }
}
