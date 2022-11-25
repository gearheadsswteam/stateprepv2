package org.firstinspires.ftc.teamcode.util;
import java.util.ArrayList;
public class ProfileChain extends MotionProfile {
    ArrayList<MotionProfile> profiles = new ArrayList<>();
    public ProfileChain(ArrayList<MotionProfile> profiles) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (profiles.get(i).tf != profiles.get(i + 1).ti) {
                throw new java.lang.RuntimeException("TimeContinuityError");
            } else if (profiles.get(i).xf != profiles.get(i + 1).xi) {
                throw new java.lang.RuntimeException("PositionContinuityError");
            } else if (profiles.get(i).vf != profiles.get(i + 1).vi) {
                throw new java.lang.RuntimeException("VelocityContinuityError");
            }
        }
        this.profiles = profiles;
        this.tf = profiles.get(profiles.size() - 1).tf;
        this.xf = profiles.get(profiles.size() - 1).xf;
        this.vf = profiles.get(profiles.size() - 1).vf;
    }
    public ProfileChain() {}
    public double getX(double t) {
        if (t < profiles.get(0).getT()) {
            return profiles.get(0).getX(t);
        }
        for (int i = 1; i < profiles.size() - 1; i++) {
            if (profiles.get(i - 1).getT() <= t && t < profiles.get(i).getT()) {
                return profiles.get(i).getX(t);
            }
        }
        return profiles.get(profiles.size() - 1).getX(t);
    }
    public double getV(double t) {
        if (t < profiles.get(0).getT()) {
            return profiles.get(0).getV(t);
        }
        for (int i = 1; i < profiles.size() - 1; i++) {
            if (profiles.get(i - 1).getT() <= t && t < profiles.get(i).getT()) {
                return profiles.get(i).getV(t);
            }
        }
        return profiles.get(profiles.size() - 1).getV(t);
    }
    public double getT() {
        return tf;
    }
    public ArrayList<MotionProfile> getProfiles() {
        return profiles;
    }
    public ProfileChain add(MotionProfile newProfile) {
        if (profiles.size() > 0) {
            if (tf != newProfile.ti) {
                throw new java.lang.RuntimeException("TimeContinuityError");
            } else if (xf != newProfile.xi) {
                throw new java.lang.RuntimeException("PositionContinuityError");
            } else if (vf != newProfile.vi) {
                throw new java.lang.RuntimeException("VelocityContinuityError");
            }
        }
        profiles.add(newProfile);
        tf = newProfile.tf;
        xf = newProfile.xf;
        vf = newProfile.vf;
        return new ProfileChain(profiles);
    }
    public ProfileChain add(ProfileChain newProfiles) {
        if (profiles.size() > 0) {
            if (tf != newProfiles.getProfiles().get(0).ti) {
                throw new java.lang.RuntimeException("TimeContinuityError");
            } else if (xf != newProfiles.getProfiles().get(0).xi) {
                throw new java.lang.RuntimeException("PositionContinuityError");
            } else if (vf != newProfiles.getProfiles().get(0).vi) {
                throw new java.lang.RuntimeException("VelocityContinuityError");
            }
        }
        profiles.addAll(newProfiles.getProfiles());
        tf = newProfiles.tf;
        xf = newProfiles.xf;
        vf = newProfiles.vf;
        return new ProfileChain(profiles);
    }
    public ProfileChain addExtendTrapezoidal(double vm, double am, double xf, double vf) {
        return add(extendTrapezoidal(vm, am, xf, vf));
    }
    public ProfileChain addExtendDelay(double tFinal) {
        return add(extendDelay(tFinal));
    }
}
