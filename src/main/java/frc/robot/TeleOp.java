package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.base.OpMode;
import frc.robot.hardware.SwerveBase;
import frc.robot.hardware.SwerveModule;
import frc.robot.util.Encoder;
import frc.robot.util.Gamepad;
import frc.robot.util.Motor;
import frc.robot.util.PID;

public class TeleOp extends OpMode {

    private SwerveBase driveBase;
    private PigeonIMU gyro;
    private Gamepad driverController;

    @Override
    public void init() {
        Motor lm1 = new Motor("LM1", 10, Motor.MotorTypes.NEO, false, 80, 0);
        Motor lm2 = new Motor("LM2", 15, Motor.MotorTypes.NEO, false, 80, 0);
        Motor rm1 = new Motor("RM1", 20, Motor.MotorTypes.NEO, false, 80, 0);
        Motor rm2 = new Motor("RM2", 25, Motor.MotorTypes.NEO, false, 80, 0);

        Motor lpm1 = new Motor("LPM1", 11, Motor.MotorTypes.NEO, false, 60, 97); // Counts per degree, ratio is 8.53
        Motor lpm2 = new Motor("LPM2", 16, Motor.MotorTypes.NEO, false, 60, 97);
        Motor rpm1 = new Motor("RPM1", 21, Motor.MotorTypes.NEO, false, 60, 97);
        Motor rpm2 = new Motor("RPM2", 27, Motor.MotorTypes.NEO, false, 60, 97);

        Encoder le1 = new Encoder("LE1", 12, Encoder.EncoderTypes.CTRE_CAN_ABS, false);
        Encoder le2 = new Encoder("LE2", 17, Encoder.EncoderTypes.CTRE_CAN_ABS, false);
        Encoder re1 = new Encoder("RE1", 22, Encoder.EncoderTypes.CTRE_CAN_ABS, false);
        Encoder re2 = new Encoder("RE2", 27, Encoder.EncoderTypes.CTRE_CAN_ABS, false);

        PID turnPID = new PID(0.075, 0, -0.000000000000000000001);
        PID drivePID = new PID(0, 0, 0);

        driveBase = new SwerveBase(new SwerveModule(lm1, drivePID, lpm1, turnPID, le1), new SwerveModule(lm2, drivePID, lpm2, turnPID, le2),
                new SwerveModule(rm1, drivePID, rpm1, turnPID, re1), new SwerveModule(rm2, drivePID, rpm2, turnPID, re2), 27, 36);

        this.gyro = new PigeonIMU(4);
        this.gyro.setYaw(0);

        this.driverController = new Gamepad(0);

        SmartDashboard.putBoolean("FieldCentricDriving", true);
    }

    @Override
    public void loop() {
        if (SmartDashboard.getBoolean("FieldCentricDriving", true)) {
            driveBase.strafeDriveFieldCentric(driverController.getRawAxis(Gamepad.Axis.kLeftX.getPort()), driverController.getRawAxis(Gamepad.Axis.kLeftY.getPort()), driverController.getRawAxis(Gamepad.Axis.kRightX.getPort()), getRobotAngle());//Add Gyro values
        } else {
            driveBase.strafeDriveV1(driverController.getRawAxis(Gamepad.Axis.kLeftX.getPort()), driverController.getRawAxis(Gamepad.Axis.kLeftY.getPort()), driverController.getRawAxis(Gamepad.Axis.kRightX.getPort()));
        }
    }

    public Rotation2d getRobotAngle() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        Rotation2d angle = Rotation2d.fromDegrees(ypr[0]).minus(Rotation2d.fromDegrees(360));
        SmartDashboard.putNumber("Robot Angle", angle.getDegrees());
        return angle;
    }

    @Override
    public void exit() {
        driveBase.setAllSpeed(0);
    }
}
