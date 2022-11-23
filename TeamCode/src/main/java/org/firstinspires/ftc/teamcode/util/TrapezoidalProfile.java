package org.firstinspires.ftc.teamcode.util;
import static java.lang.Math.*;
public class TrapezoidalProfile {
    double am;
    double vm;
    double xi;
    double vi;
    double ti;
    double xf;
    double tf;
    boolean flat;
    public TrapezoidalProfile(double vm, double am, double xi, double vi, double ti, double xf) {
        this.am = am;
        this.vm = vm;
        this.xi = xi;
        this.vi = vi;
        this.ti = ti;
        this.xf = xf;
        if (xf > xi) {
            if (2 * pow(vm, 2) - pow(vi, 2) < 2 * am * (xf - xi)) {
                this.flat = true;
                this.tf = ti + (xf - xi) / vm + (vm - vi) / am + pow(vi, 2) / (2 * am * vm);
            } else {
                this.flat = false;
                this.tf = ti + (2 * sqrt(am * (xf - xi) + pow(vi, 2) / 2) - vi) / am;
            }
        } else {
            if (2 * pow(vm, 2) - pow(vi, 2) < 2 * am * (xi - xf)) {
                this.flat = true;
                this.tf = ti + (xi - xf) / vm + (vm + vi) / am + pow(vi, 2) / (2 * am * vm);
            } else {
                this.flat = false;
                this.tf = ti + (2 * sqrt(am * (xi - xf) + pow(vi, 2)) + vi) / 2;
            }
        }
    }
    public double getX(double t) {
        if (t < tf) {
            if (xf > xi && flat) {
                if (t < ti + (vm - vi) / am) {
                    return xi + vi * (t - ti) + am * pow(t - ti, 2) / 2;
                } else if (t < tf - vm / am) {
                    return xi + vm * (t - ti) - pow(vm - vi, 2) / (2 * am);
                } else {
                    return xf - am * pow(tf - t, 2) / 2;
                }
            } else if (xf > xi) {
                if (t < tf / 2 - vi / (2 * am)) {
                    return xi + vi * (t - ti) + am * pow(t - ti, 2) / 2;
                } else {
                    return xf - am * pow(tf - t, 2) / 2;
                }
            } else if (flat) {
                if (t < ti + (vm + vi) / am) {
                    return xi + vi * (t - ti) - am * pow(t - ti, 2) / 2;
                } else if (t < tf - vm / am) {
                    return xi - vm * (t - ti) + pow(vm + vi, 2) / (2 * am);
                } else {
                    return xf + am * pow(tf - t, 2) / 2;
                }
            } else {
                if (t < tf / 2 + vi / (2 * am)) {
                    return xi + vi * (t - ti) - am * pow(t - ti, 2) / 2;
                } else {
                    return xf + am * pow(tf - t, 2) / 2;
                }
            }
        } else {
            return xf;
        }
    }
    public double getV(double t) {
        if (t < tf) {
            if (xf > xi && flat) {
                if (t < (vm - vi) / am) {
                    return vi + am * t;
                } else if (t < tf - vm / am) {
                    return vm;
                } else {
                    return (tf - t) * am;
                }
            }
        } else {
            return 0;
        }
    }
    public boolean flat() {
        return flat;
    }
}