package org.firstinspires.ftc.teamcode.classes;
import com.acmerobotics.roadrunner.geometry.Pose2d;
public class ValueStorage {
    public static int holderMinCount = 100000;
    public static int signalMinCount = 10;
    public static double holderDetectionThreshold = 0.75;
    public static double rollerDown = 0.48;
    public static double rollerUp = 0.25;
    public static double rollerRetract = 0.25;
    public static double gripperHold = 0.13;
    public static double gripperRelease = 0.32;
    public static double armRest = 0.06;
    public static double armUp = 0.55;
    public static double armIn = 0.40;
    public static double wristRest = 0.71;
    public static double wristUp = 0.20;
    public static double wristIn = 0.53;
    public static double armOffset = 1.01;
    public static double wristOffset = 1.10;
    public static double liftMaxAccel = 3000;
    public static double liftMaxVel = 1400;
    public static double liftKp = 0.05;
    public static double liftKi = 0;
    public static double liftKd = 0;
    public static double armMaxVel = 0.66;
    public static double armMaxAccel = 5;
    public static double wristMaxVel = 2;
    public static double wristMaxAccel = 20;
    public static double liftKf (double input) {
        if (input < 10) {
            return -0.2;
        } else {
            return 0.0003 * input;
        }
    }
    public static ProfileChain forwardArmProfile1(double t) {
        return new ProfileChain()
                .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, 0.5 + t, armRest, 0, armUp, 0));
    }
    public static ProfileChain forwardWristProfile1(double t) {
        return new ProfileChain()
                .add(new TrapezoidalProfile(0.5, wristMaxAccel, 0.5 + t, wristRest, 0, wristIn, 0))
                .addExtendDelay(t + 1.2)
                .addExtendTrapezoidal(wristMaxVel, wristMaxAccel,   wristUp, 0);
    }
    public static ProfileChain backArmProfile1(double t) {
        return new ProfileChain()
                .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, t, armUp, 0, armRest, 0));
    }
    public static ProfileChain backWristProfile1(double t) {
        return new ProfileChain()
                .add(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, t, wristUp, 0, wristIn, 0))
                .addExtendDelay(t + 0.5)
                .addExtendTrapezoidal(0.5, wristMaxAccel, wristRest, 0);
    }
    public static ProfileChain forwardArmProfile2(double t) {
        return new ProfileChain()
                .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, t, armIn, 0, armRest, 0));
    }
    public static ProfileChain forwardWristProfile2(double t) {
        return new ProfileChain()
                .add(new TrapezoidalProfile(0.5, wristMaxAccel, 0.3 + t, wristIn, 0, wristRest, 0));
    }
    public static ProfileChain autonomousArmProfile(double t) {
        return new ProfileChain()
                .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, t, armIn, 0, armUp, 0));
    }
    public static ProfileChain autonomousWristProfile(double t) {
        return new ProfileChain()
                .add(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, t + 0.5, wristIn, 0, wristUp, 0));
    }
    public static double[] liftLowClose = {150, armUp, wristUp};
    public static double[] liftMedClose = {600, armUp, wristUp};
    public static double[] liftHighClose = {1000, armUp, wristUp};
    public static double[] liftHighFar = {1050, 0.70, wristUp};
    public static double[] liftGroundClose = {0, 0.97, 0.30};
    public static int redMultiplier = 1;
    public static int caseDetectionThreshold = 10;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}