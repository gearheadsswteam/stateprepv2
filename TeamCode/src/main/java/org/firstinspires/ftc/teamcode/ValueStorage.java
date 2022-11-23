package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
public class ValueStorage {
    public static int holderConfidence = 10;
    public static int signalConfidence = 10;
    public static double holderDetectionThreshold = 2;
    public static double rollerDown = 0.5;
    public static double rollerUp = 0.25;
    public static double rollerRetract = 0.25;
    public static double gripperHold = 0.46;
    public static double gripperRelease = 0.70;
    public static double armRest;
    public static double armUp;
    public static double armDown;
    public static double wristRest = 0.73;
    public static double wristUp;
    public static double wristDown;
    public static double armOffset;
    public static double wristOffset;
    public static double liftKp = 0.01;
    public static double liftKi = 0;
    public static double liftKd = 0;
    public static double liftKf (double input) {
        return 0;
    }
    public static double[] forwardSafeArmPos1(double t) {
        return new double[] {0, 0, 0};
    }
    public static double[] backSafeArmPos1(double t) {
        return new double[] {0, 0, 0};
    }
    public static double[] forwardSafeArmPos2(double t) {
        return new double[] {0, 0, 0};
    }
    public static double[] forwardStateTimes(double liftPos, double armPos, double wristPos) {
        return new double[] {500, 1500, 5000, 500, 5000};
    }
    public static double[] backStateTimes = {1000, 0, 500, 0, 500};
    public static double[] liftLow = {0, 0, 0};
    public static double[] liftMed = {0, 0, 0};
    public static double[] liftHigh = {0, 0, 0};
    public static double[] liftGround = {0, 0, 0};
    public static int redMultiplier = 1;
    public static int caseDetectionThreshold = 10;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}