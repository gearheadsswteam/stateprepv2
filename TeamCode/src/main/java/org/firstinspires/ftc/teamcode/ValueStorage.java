package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.ProfileChain;
import org.firstinspires.ftc.teamcode.util.TrapezoidalProfile;

import java.util.ArrayList;
import java.util.Arrays;

public class ValueStorage {
    public static int holderConfidence = 10;
    public static int signalConfidence = 10;
    public static double holderDetectionThreshold = 2;
    public static double rollerDown = 0.5;
    public static double rollerUp = 0.25;
    public static double rollerRetract = 0.25;
    public static double gripperHold = 0.46;
    public static double gripperRelease = 0.70;
    public static double armRest = 0.12;
    public static double armUp = 0.61;
    public static double armIn = 0.46;
    public static double wristRest = 0.73;
    public static double wristUp = 0.22;
    public static double wristIn = 0.55;
    public static double armOffset = 0.98;
    public static double wristOffset = 1.01;
    public static double liftMaxAccel = 5000;
    public static double liftMaxVel = 1400;
    public static double liftKp = 0.05;
    public static double liftKi = 0;
    public static double liftKd = 0;
    public static double armMaxVel = 0.65;
    public static double armMaxAccel = 5;
    public static double wristMaxVel = 0.75;
    public static double wristMaxAccel = 5;
    public static double liftKf (double input) {
        return 0.0003 * input;
    }
    public static ProfileChain forwardSafeArmPos1 = new ProfileChain()
            .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, 0.5, armRest, 0, armUp, 0));
    public static ProfileChain forwardSafeWristPos1 = new ProfileChain()
            .add(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, 0.5, wristRest, 0, 0.55, 0))
            .addExtendDelay(1.2)
            .addExtendTrapezoidal(wristMaxVel, wristMaxAccel, wristUp, 0);
    public static ProfileChain backSafeArmPos1 = new ProfileChain();
    public static ProfileChain backSafeWristPos1 = new ProfileChain();
    public static ProfileChain forwardSafeArmPos2 = new ProfileChain();
    public static ProfileChain forwardSafeWristPos2 = new ProfileChain();
    public static double[] liftLow = {0, armUp, wristUp};
    public static double[] liftMed = {0, armUp, wristUp};
    public static double[] liftHigh = {0, armUp, wristUp};
    public static double[] liftGround = {0, 1.00, 0.32};
    public static int redMultiplier = 1;
    public static int caseDetectionThreshold = 10;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}