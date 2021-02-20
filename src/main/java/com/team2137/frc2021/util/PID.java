package com.team2137.frc2021.util;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class PID {

    double P, I, D, IZ, FF;
    double S, V, A = 0;

    public PID (double _P, double _I, double _D) {
        this.P = _P;
        this.I = _I;
        this.D = _D;
    }

    public PID (double _P, double _I, double _D, double _FF, double _IZ) {
        this(_P, _I, _D);
        this.IZ = _IZ;
        this.FF = _FF;
    }

    public PID (double _P, double _I, double _D, double _S, double _V, double _A) {
        this(_P, _I, _D);
        S = _S;
        V = _V;
        A = _A;
    }

    public PID (double _P, double _I, double _D, double _FF, double _IZ, double _S, double _V, double _A) {
        this(_P, _I, _D, _S, _V, _A);
        FF = _FF;
        IZ = _IZ;
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

    public double getS() {
        return S;
    }

    public void setS(double s) {
        S = s;
    }

    public double getV() {
        return V;
    }

    public void setV(double v) {
        V = v;
    }

    public double getA() {
        return A;
    }

    public void setA(double a) {
        A = a;
    }

    public double[] getPIDArray() {
        return new double[] {P, I, D};
    }

    public PIDController getWPIPIDController() {
        return new PIDController(P, I, D);
    }

    public SimpleMotorFeedforward getWPIFeedForwardController() {
        return new SimpleMotorFeedforward(S, V, A);
    }
}
