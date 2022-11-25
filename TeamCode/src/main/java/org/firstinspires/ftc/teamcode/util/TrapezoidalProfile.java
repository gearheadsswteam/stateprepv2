package org.firstinspires.ftc.teamcode.util;
import static java.lang.Math.*;
public class TrapezoidalProfile extends MotionProfile {
    double am;
    double vm;
    boolean flat;
    public TrapezoidalProfile(double vm, double am, double ti, double xi, double vi, double xf, double vf) {
        this.am = am;
        this.vm = vm;
        this.xi = xi;
        this.vi = vi;
        this.ti = ti;
        this.xf = xf;
        this.vf = vf;
        if (abs(pow(vf, 2) - pow(vi, 2)) / (2 * am) > abs(xf - xi)) {
            throw new java.lang.RuntimeException("ImpossibleProfileError");
        }
        this.flat = 2 * pow(vm, 2) - pow(vi, 2) - pow(vf, 2) < 2 * am * abs(xf - xi);
        if (xf > xi && flat) {
            this.tf = ti + (xf - xi) / vm + (pow(vm - vi, 2) + pow(vm - vf, 2)) / (2 * am * vm);
        } else if (xf > xi) {
            this.tf = ti + (2 * sqrt(am * (xf - xi) + (pow(vi, 2) + pow(vm, 2))/ 2) - vi - vf) / am;
        } else if (flat) {
            this.tf = ti + (xi - xf) / vm + (pow(vm - vi, 2) + pow(vm - vf, 2)) / (2 * am * vm);
        } else {
            this.tf = ti + (2 * sqrt(am * (xi - xf) + (pow(vi, 2) + pow(vf, 2)) / 2) + vi + vf) / am;
        }
    }
    public double getX(double t) {
        if (t < ti) {
            return xi + vi * (t - ti);
        } if (t < tf) {
            if (xf > xi && flat) {
                if (t < ti + (vm - vi) / am) {
                    return xi + vi * (t - ti) + am * pow(t - ti, 2) / 2;
                } else if (t < tf - vm / am) {
                    return xi + vm * (t - ti) - pow(vm - vi, 2) / (2 * am);
                } else {
                    return xf - vf * (tf - t) - am * pow(tf - t, 2) / 2;
                }
            } else if (xf > xi) {
                if (t < (tf + ti) / 2 - vi / (2 * am)) {
                    return xi + vi * (t - ti) + am * pow(t - ti, 2) / 2;
                } else {
                    return xf - vf * (tf - t) * am * pow(tf - t, 2) / 2;
                }
            } else if (flat) {
                if (t < ti + (vm + vi) / am) {
                    return xi + vi * (t - ti) - am * pow(t - ti, 2) / 2;
                } else if (t < tf - vm / am) {
                    return xi - vm * (t - ti) + pow(vm + vi, 2) / (2 * am);
                } else {
                    return xf - vf * (tf - t) + am * pow(tf - t, 2) / 2;
                }
            } else {
                if (t < (tf + ti) / 2 + vi / (2 * am)) {
                    return xi + vi * (t - ti) - am * pow(t - ti, 2) / 2;
                } else {
                    return xf - vf * (tf - t) + am * pow(tf - t, 2) / 2;
                }
            }
        } else {
            return xf + vf * (t - tf);
        }
    }
    public double getV(double t) {
        if (t < ti) {
            return vi;
        } else if (t < tf) {
            if (xf > xi && flat) {
                if (t < ti + (vm - vi) / am) {
                    return vi + am * (t - ti);
                } else if (t < tf - vm / am) {
                    return vm;
                } else {
                    return vf + (tf - t) * am;
                }
            } else if (xf > xi) {
                if (t < (tf + ti) / 2 + (vf - vi) / (2 * am)) {
                    return vi + am * (t - ti);
                } else {
                    return vf + am * (tf - t);
                }
            } else if (flat) {
                if (t < ti + (vm + vi) / am) {
                    return vi - am * (t - ti);
                } else if (t < tf - vm / am) {
                    return -vm;
                } else {
                    return vf - am * (tf - t);
                }
            } else {
                if (t < (tf + ti) / 2 + (vi - vf) / (2 * am)) {
                    return vi - am * (t - ti);
                } else {
                    return vf - am * (tf - t);
                }
            }
        } else {
            return vf;
        }
    }
    public TrapezoidalProfile extendTrapezoidal(double t, double xFinal, double vFinal) {
        return new TrapezoidalProfile(vm, am, t, getX(t), getV(t), xFinal, vFinal);
    }
    public TrapezoidalProfile extendTrapezoidal(double xFinal, double vFinal) {
        return new TrapezoidalProfile(vm, am, tf, xf, vf, xFinal, vFinal);
    }
}