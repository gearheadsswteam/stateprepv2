package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Scalar;
import java.util.ArrayList;
import java.util.Arrays;
public class VisionValueStorage {
    public static int signalMinArea = 5000;
    public static Scalar[] signalLower = {new Scalar(0, 0, 0), new Scalar(0, 140, 130), new Scalar(0, 0, 140)};
    public static Scalar[] signalUpper = {new Scalar(255, 120, 130), new Scalar(255, 255, 255), new Scalar(255, 110, 255)};
    public static double tagSize = 0.052;
    public static ArrayList<Integer> tagIds = new ArrayList<>(Arrays.asList(1, 2, 3));
}
