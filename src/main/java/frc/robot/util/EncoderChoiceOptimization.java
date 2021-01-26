package frc.robot.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;

import java.util.*;
import java.util.function.BiConsumer;

import static frc.robot.util.Constants.getAverage;

public class EncoderChoiceOptimization extends Thread {

    private CANEncoder motorEncoder;
    private CANCoder otherEncoder;

    private volatile List<Map.Entry<Double, Double>> encoderValues;

    private volatile double stdMotorDerivative = 0;
    private volatile double stdOtherDerivative = 0;
    private double motorEncoderZeroPosition = 0;
    private double motorEncoderCountsPerDegree = 0;
    private double maxVeriance = 0;

    private int maxRecordLength = 0;

    private volatile boolean exit = false;

    public EncoderChoiceOptimization (CANEncoder _motorEncoder, CANCoder _otherEncoder, double _motorEncoderCountsPerDegree, double _MaxVariance, int _recordLength) {
        motorEncoder = _motorEncoder;
        otherEncoder = _otherEncoder;
        maxVeriance = _MaxVariance;
        maxRecordLength = _recordLength;
        motorEncoderCountsPerDegree = _motorEncoderCountsPerDegree;
        motorEncoderZeroPosition = motorEncoder.getPosition() - (((otherEncoder.getAbsolutePosition() / 4096) * 360) * motorEncoderCountsPerDegree); //TODO fix config on the motor

        this.start();
    }

    public double getOptimalEncoderReading() {
        if (stdOtherDerivative > maxVeriance) {
            return ((motorEncoder.getPosition() - motorEncoderZeroPosition) / motorEncoderCountsPerDegree) % 360;
        } else {
            return otherEncoder.getAbsolutePosition();
        }
    }

    @Override
    public void run() {
        while(!exit) {
            if(encoderValues.size() > maxRecordLength) encoderValues.remove(0);
            encoderValues.add(new AbstractMap.SimpleEntry<Double, Double>(motorEncoder.getPosition(), otherEncoder.getAbsolutePosition()));

            double[] average = getAverage(encoderValues);

            double motorDirv = 0;
            double otherDirv = 0;
            for (Map.Entry<Double, Double> i : encoderValues) {
                motorDirv += Math.pow(i.getKey() - average[0], 2);
                otherDirv += Math.pow(i.getValue() - average[1], 2);
            }

            stdMotorDerivative = Math.sqrt(motorDirv / encoderValues.size());
            stdOtherDerivative = Math.sqrt(otherDirv / encoderValues.size());
        }
    }

    public double getMotorEncoderVariance() {
        return this.stdMotorDerivative;
    }

    public double getOtherEncoderVariance() {
        return this.stdOtherDerivative;
    }

    public void resetListener() {
        this.encoderValues.clear();
        this.encoderValues.add(new AbstractMap.SimpleEntry<Double, Double>(motorEncoder.getPosition(), otherEncoder.getAbsolutePosition()));
    }

    public void endListener() {
        this.exit = false;
    }
}
