package com.team2137.frc2021.util;

public class PID {

    double P, I, D, IZ, FF;

    public PID (double _P, double _I, double _D) {
        this.P = _P;
        this.I = _I;
        this.D = _D;
    }

    public PID (double _P, double _I, double _D, double _IZ, double _FF) {
        this(_P, _I, _D);
        this.IZ = _IZ;
        this.FF = _FF;
    }

    public double getP() {
        return P;
    }

    public void setP(double p) {
        P = p;
    }

    public double getI() {
        return I;
    }

    public void setI(double i) {
        I = i;
    }

    public double getD() {
        return D;
    }

    public void setD(double d) {
        D = d;
    }

    public double getIZ() {
        return IZ;
    }

    public void setIZ(double IZ) {
        this.IZ = IZ;
    }

    public double getFF() {
        return FF;
    }

    public void setFF(double FF) {
        this.FF = FF;
    }
}
