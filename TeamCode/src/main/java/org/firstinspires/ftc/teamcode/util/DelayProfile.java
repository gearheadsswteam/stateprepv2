package org.firstinspires.ftc.teamcode.util;
public class DelayProfile extends MotionProfile {
    public DelayProfile(double ti, double xi, double vi, double tf) {
        this.ti = ti;
        this.xi = xi;
        this.vi = vi;
        this.tf = tf;
        this.xf = xi + vi * (tf - ti);
        this.vf = vi;
    }
    public double getX(double t) {
        return xi + vi * (t - ti);
    }
    public double getV(double t) {
        return vi;
    }
}
