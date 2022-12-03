package org.firstinspires.ftc.teamcode.classes;
import java.util.ArrayList;
public class ProfileChain extends MotionProfile {
    ArrayList<MotionProfile> profiles = new ArrayList<>();
    public ProfileChain(ArrayList<MotionProfile> profiles) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (profiles.get(i).getX(profiles.get(i + 1).getTi()) != profiles.get(i + 1).xi) {
                throw new java.lang.RuntimeException("PositionContinuityError");
            } else if (profiles.get(i).getV(profiles.get(i + 1).getTi()) != profiles.get(i + 1).vi) {
                throw new java.lang.RuntimeException("VelocityContinuityError");
            }
        }
        this.profiles = profiles;
        this.tf = profiles.get(profiles.size() - 1).getTf();
        this.xf = profiles.get(profiles.size() - 1).xf;
        this.vf = profiles.get(profiles.size() - 1).vf;
    }
    public ProfileChain(MotionProfile profile) {
        this.profiles.add(profile);
        this.tf = profile.getTf();
        this.xf = profile.xf;
        this.vf = profile.vf;
    }
    public ProfileChain() {}
    @Override
    public double getX(double t) {
        for (int i = 0; i < profiles.size() - 2; i++) {
            if (t < profiles.get(i + 1).getTi()) {
                return profiles.get(i).getX(t);
            }
        }
        return profiles.get(profiles.size() - 1).getX(t);
    }
    @Override
    public double getV(double t) {
        for (int i = 0; i < profiles.size() - 2; i++) {
            if (t < profiles.get(i + 1).getTi()) {
                return profiles.get(i).getV(t);
            }
        }
        return profiles.get(profiles.size() - 1).getV(t);
    }
    public ArrayList<MotionProfile> getProfiles() {
        return profiles;
    }
    public ProfileChain add(MotionProfile newProfile) {
        if (profiles.size() > 0) {
            if (getX(newProfile.ti) != newProfile.xi) {
                throw new java.lang.RuntimeException("PositionContinuityError");
            } else if (getV(newProfile.ti) != newProfile.vi) {
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
            if (getX(newProfiles.getProfiles().get(0).ti) != newProfiles.getProfiles().get(0).xi) {
                throw new java.lang.RuntimeException("PositionContinuityError");
            } else if (getV(newProfiles.getProfiles().get(0).ti) != newProfiles.getProfiles().get(0).vi) {
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
