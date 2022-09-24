package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.opencv.core.Scalar;
public class ValueStorage {
    public static double clawOpen = 0.60;
    public static double clawClosed = 0.86;
    public static double initTheta1 = 0;
    public static double initTheta2 = -Math.PI / 2;
    public static double countsPerRad1 = 427.9;
    public static double countsPerRad2 = 427.9;
    public static int[][] stateTimes = {{250, 500, 500, 1000}, {500, 500, 500, 500}, {250, 500, 500, 1000}, {250, 250, 250, 250,}};
    public static int[] arm1States = {0, 500, 600, 600};
    public static int[] arm2States = {150, 400, 800, 1300};
    public static int arm1Max = 700;
    public static int arm1Min = 0;
    public static int arm2Max = 1400;
    public static int arm2Min = 0;
    public static int redMultiplier = 1;
    public static int caseDetectionThreshold = 10;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
    public static Scalar[] lower;
    public static Scalar[] upper;
}