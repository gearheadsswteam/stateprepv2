package org.firstinspires.ftc.teamcode.classes;

public abstract class MotionProfile {
    double xi;
    double vi;
    double ti;
    double xf;
    double tf;
    double vf;
    public abstract double getX(double t);
    public abstract double getV(double t);
    public double getT() {
        return tf;
    }
    public TrapezoidalProfile extendTrapezoidal(double vm, double am, double t, double xf, double vf) {
        return new TrapezoidalProfile(vm, am, t, getX(t), getV(t), xf, vf);
    }
    public TrapezoidalProfile extendTrapezoidal(double vm, double am, double xf, double vf) {
        return extendTrapezoidal(vm, am, getT(), xf, vf);
    }
    public DelayProfile extendDelay(double t, double tFinal) {
        return new DelayProfile(t, getX(t), getV(t), tFinal);
    }
    public DelayProfile extendDelay(double tFinal) {
        return extendDelay(tf, tFinal);
    }
}
