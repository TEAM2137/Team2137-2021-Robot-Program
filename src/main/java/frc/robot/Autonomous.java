package frc.robot;

import frc.robot.base.OpMode;
import frc.robot.hardware.SwerveBase;
import frc.robot.hardware.SwerveModule;
import frc.robot.util.Encoder;
import frc.robot.util.Motor;
import frc.robot.util.PID;
import frc.robot.PathPlanning.LinePlot;
import frc.robot.PathPlanning.PathPlanner;

import java.awt.*;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Writer;

public class Autonomous extends OpMode {

        private SwerveBase base;
        PathPlanner path;
        private double startTime = 0;

        @Override
        public void init() {

                // path heading accumulated in degrees
                // FalconPathPlanner.print(path.heading);

                Motor lm1 = new Motor("LM1", 10, Motor.MotorTypes.NEO, false, 80, 0);
                Motor lm2 = new Motor("LM2", 15, Motor.MotorTypes.NEO, false, 80, 0);
                Motor rm1 = new Motor("RM1", 20, Motor.MotorTypes.NEO, false, 80, 0);
                Motor rm2 = new Motor("RM2", 25, Motor.MotorTypes.NEO, false, 80, 0);

                Motor lpm1 = new Motor("LPM1", 11, Motor.MotorTypes.NEO, false, 60, 97); // Counts per degree, ratio is// 8.53
                Motor lpm2 = new Motor("LPM2", 16, Motor.MotorTypes.NEO, false, 60, 97);
                Motor rpm1 = new Motor("RPM1", 21, Motor.MotorTypes.NEO, false, 60, 97);
                Motor rpm2 = new Motor("RPM2", 26, Motor.MotorTypes.NEO, false, 60, 97);

                Encoder le1 = new Encoder("LE1", 12, Encoder.EncoderTypes.CTRE_CAN_ABS, false, "400");
                Encoder le2 = new Encoder("LE2", 17, Encoder.EncoderTypes.CTRE_CAN_ABS, false, "400");
                Encoder re1 = new Encoder("RE1", 22, Encoder.EncoderTypes.CTRE_CAN_ABS, false, "400");
                Encoder re2 = new Encoder("RE2", 27, Encoder.EncoderTypes.CTRE_CAN_ABS, false, "400");

                PID turnPID = new PID(0.075, 0, -0.000000000000000000001);
                PID drivePID = new PID(0, 0, 0);

            base = new SwerveBase(new SwerveModule(lm1, drivePID, lpm1, turnPID, le1),
                    new SwerveModule(lm2, drivePID, lpm2, turnPID, le2),
                    new SwerveModule(rm1, drivePID, rpm1, turnPID, re1),
                    new SwerveModule(rm2, drivePID, rpm2, turnPID, re2), 27, 36);

            double[][] CheesyPath = new double[][]{
                    {5,0}
            };

            double totalTime = 5; //seconds
            double timeStep = 0.1; //period of control loop on Rio, seconds
            double robotTrackWidth = 3; //distance between left and right wheels, feet

            path = new PathPlanner(CheesyPath);
            path.calculate(totalTime, timeStep, robotTrackWidth);
        startTime = System.currentTimeMillis();
        base.zeroModules();
    }

    @Override
    public void loop() {
        // if (System.currentTimeMillis() - startTime < 5000) {                
        //         int pointNum = (int) (((System.currentTimeMillis() - startTime) / 10));
        //         base.setLeftVelocity(path.smoothLeftVelocity[pointNum][1], 14.4);
        //         base.setRightVelocity(path.smoothRightVelocity[pointNum][1], 14.4);
        // }
        base.zeroModules();
    }

    @Override
    public void exit() {

    }
    
}
