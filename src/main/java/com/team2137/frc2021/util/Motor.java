package com.team2137.frc2021.util;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMaxLowLevel.*;

import javax.annotation.Nullable;

public class Motor {
    String name;
    int id;
    MotorTypes type;
    boolean inverted;
    String[] parms;
    int currentLimit;
    double gearRatio = 1;
    PID pidValues;

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
    public Motor(String _name, int _id, MotorTypes _type, boolean _invert, int _currentLimit, double _gearRatio, @Nullable PID pid, String... _parms) {
        this.name = _name;
        this.id = _id;
        this.type = _type;
        this.inverted = _invert;
        this.parms = _parms;
        this.currentLimit = _currentLimit;
        this.gearRatio = _gearRatio;
        if (pid != null)
            this.pidValues = pid;
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