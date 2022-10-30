package org.firstinspires.ftc.teamcode.vision;
import org.opencv.core.Scalar;
public class VisionValueStorage {
    public static int signalMinArea = 5000;
    public static Scalar[] signalLower = {new Scalar(0, 0, 0), new Scalar(0, 140, 130), new Scalar(0, 0, 140)};
    public static Scalar[] signalUpper = {new Scalar(255, 120, 130), new Scalar(255, 255, 255), new Scalar(255, 110, 255)};
}
