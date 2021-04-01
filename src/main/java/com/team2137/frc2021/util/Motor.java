package com.team2137.frc2021.util;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMaxLowLevel.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import javax.annotation.Nullable;

public class Motor {
    private String name;
    private int id;
    private MotorTypes type;
    private boolean inverted;
    private String[] parms;
    private int currentLimit;
    private double gearRatio = 1;
    private double rampRate = 0;
    private PID pidValues;

    /**
     * Create a new Motor Object for debug and better storage
     * @param _name
     * @param _id
     * @param _type
     * @param _invert
     * @param _currentLimit
     * @param _gearRatio
     * @param _parms
     */
    public Motor(String _name, int _id, MotorTypes _type, boolean _invert, int _currentLimit, double _gearRatio, String... _parms) {
        this.name = _name;
        this.id = _id;
        this.type = _type;
        this.inverted = _invert;
        this.parms = _parms;
        this.currentLimit = _currentLimit;
        this.gearRatio = _gearRatio;
    }

    public Motor(String _name, int _id, MotorTypes _type, boolean _invert, int _currentLimit, double _gearRatio, PID pid, String... _parms) {
        this.name = _name;
        this.id = _id;
        this.type = _type;
        this.inverted = _invert;
        this.parms = _parms;
        this.currentLimit = _currentLimit;
        this.gearRatio = _gearRatio;
        this.pidValues = pid;
    }

    public Motor(String _name, int _id, MotorTypes _type, boolean _invert, int _currentLimit, double _gearRatio, double _rampRate, @Nullable PID _pid, String... _parms) {
        this(_name, _id, _type, _invert, _currentLimit, _gearRatio, _parms);
        this.rampRate = _rampRate;
        if (_pid != null)
            this.pidValues = _pid;
    }

    public double getGearRatio() {
        return this.gearRatio;
    }

    public String getMotorName() {
        return this.name;
    }

    public int getMotorID() {
        return this.id;
    }

    public MotorTypes getMotorType() {
        return this.type;
    }

    public String[] getParms() {
        return this.parms;
    }

    public String getParm(int i) {
        return this.parms[i];
    }

    public Double getParmDouble(int i) {
        return Double.parseDouble(this.parms[1]);
    }

    public boolean inverted() {
        return this.inverted;
    }

    public int getCurrentLimit() {
        return this.currentLimit;
    }

    public PID getPID() {
        return this.pidValues;
    }

    public PIDController getMotorPIDController() {
        return this.pidValues.getWPIPIDController();
    }

    public SimpleMotorFeedforward getMotorFeedForwardController() {
        return this.pidValues.getWPIFeedForwardController();
    }

    public double getRampRate() {
        return rampRate;
    }

    public void setRampRate(double rampRate) {
        this.rampRate = rampRate;
    }

    @Override
    public String toString() {
        return "Name: " + name + "\nID: " + id + "\nType: " + type.toString() + "\nInverted: " + inverted + "\nCurrent Limit: " + currentLimit
                + "\nGear Ratio: " + gearRatio + "\nRamp Rate: " + rampRate + "\nPID: " + pidValues.toString();
    }

    public enum MotorTypes {
        NEO (MotorType.kBrushless),
        NEO550 (MotorType.kBrushless),
        FALCON (MotorType.kBrushless),
        BAG (MotorType.kBrushed),
        CIM (MotorType.kBrushed);

        MotorType type = MotorType.kBrushless;

        MotorTypes (MotorType revType) {
            type = revType;
        }

        public MotorType getREVType() {
            return this.type;
        }
    }
}