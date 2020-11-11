package frc.robot.base;

public class Feedforward { 
    private double mS = 0.0;
    private double mV = 0.0;
    private double mA = 0.0;

    public Feedforward (double s, double v, double a) {
        this.mS = s;
        this.mV = v;
        this.mA = a;
    }

    public double calculate(double velocity, double acceleration) {
        return this.mS * Math.signum(velocity) + this.mV * velocity + this.mA * acceleration;
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    public void setSVA(double s, double v, double a) {
        this.mS = s;
        this.mV = v;
        this.mA = a;
    }
}