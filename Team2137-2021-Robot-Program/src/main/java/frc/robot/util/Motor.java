package frc.robot.util;

import frc.robot.Constants.MotorTypes;

public class Motor {
    String name;
    int id;
    MotorTypes type;
    boolean inverted;
    String[] parms;

    public Motor(String _name, int _id, MotorTypes _type, boolean _invert, String... _parms) {
        this.name = _name;
        this.id = _id;
        this.type = _type;
        this.inverted = _invert;
        this.parms = _parms;
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

    public boolean inverted() {
        return this.inverted;
    }
}