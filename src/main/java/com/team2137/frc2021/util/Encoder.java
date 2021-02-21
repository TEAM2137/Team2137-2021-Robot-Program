package com.team2137.frc2021.util;

public class Encoder {

    public enum EncoderTypes {
        CTRE_CAN_ABS
    }

    String name;
    int id;
    EncoderTypes type;
    boolean inverted;
    String[] parms;

    public Encoder(String _name, int _id, EncoderTypes _type, boolean _invert, String... _parms) {
        this.name = _name;
        this.id = _id;
        this.type = _type;
        this.inverted = _invert;
        this.parms = _parms;
    }

    public String getMotorName() {
        return this.name;
    }

    public int getEncoderID() {
        return this.id;
    }

    public EncoderTypes getEncoderType() {
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
