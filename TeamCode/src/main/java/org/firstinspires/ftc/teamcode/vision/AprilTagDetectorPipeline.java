package org.firstinspires.ftc.teamcode.vision;
import static org.firstinspires.ftc.teamcode.vision.VisionValueStorage.*;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
public class AprilTagDetectorPipeline extends OpenCvPipeline {
    int caseDetected = 0;
    long tagDetectorPointer = AprilTagDetectorJNI.createApriltagDetector("tag36h11", 3, 1);
    ArrayList<AprilTagDetection> detections;
    Mat grayscale;
    Mat output;
    @Override
    public Mat processFrame(Mat input) {
        output = input.clone();
        Imgproc.cvtColor(input, grayscale, Imgproc.COLOR_RGB2GRAY);
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(tagDetectorPointer, grayscale, tagSize, 1, 1, 1, 1);
        if (!detections.isEmpty() && tagIds.contains(detections.get(0).id)) {
            caseDetected = tagIds.indexOf(detections.get(0).id) + 1;
            Imgproc.rectangle(output, detections.get(0).corners[0], detections.get(0).corners[2], new Scalar(0, 0, 255), 2);
            Imgproc.putText(output, "Case: " + caseDetected, new Point(10, 350), 0, 0.5, new Scalar(0, 0, 255), 1);
            Imgproc.putText(output, "Tag ID: " + detections.get(0).id, new Point(10, 335), 0, 0.5, new Scalar(0, 0, 255), 1);
        } else {
            caseDetected = 0;
            Imgproc.putText(output, "No Case Detected", new Point(10, 350), 0, 0.5, new Scalar(0, 0, 255), 1);
            if (detections.isEmpty()) {
                Imgproc.putText(output, "No Tag Detected", new Point(10, 335), 0, 0.5, new Scalar(0, 0, 255), 1);
            } else {
                Imgproc.putText(output, "Tag ID: " + detections.get(0).id, new Point(10, 335), 0, 0.5, new Scalar(0, 0, 255), 1);
            }
        }
        return output;
    }
    public void end() {
        AprilTagDetectorJNI.releaseApriltagDetector(tagDetectorPointer);
        grayscale.release();
        output.release();
    }
    public int getCaseDetected() {
        return caseDetected;
    }
}