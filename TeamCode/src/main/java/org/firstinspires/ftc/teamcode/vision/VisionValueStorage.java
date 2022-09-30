package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Scalar;
public class VisionValueStorage {
    public static int signalMinArea = 5000;
    public static Scalar[] signalLower = {new Scalar(0, 0, 0), new Scalar(0, 140, 140), new Scalar(0, 0, 110)};
    public static Scalar[] signalUpper = {new Scalar(255, 120, 120), new Scalar(255, 255, 255), new Scalar(255, 90, 255)};
}
