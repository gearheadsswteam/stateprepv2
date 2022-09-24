package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
public class SignalDetector {
    OpenCvCamera camera;
    OpenCvPipeline pipeline;
    Scalar[] lower;
    Scalar[] upper;
    int caseDetected = 2;
    public SignalDetector(HardwareMap hwMap, Scalar[] lower, Scalar[] upper) {
        int id = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName name = hwMap.get(WebcamName.class, "camera");
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(name, id);
        this.lower = lower;
        this.upper = upper;
        this.pipeline = new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                Mat output = input.clone();
                int[] maxColor = {0, -1};
                for (int i = 0; i < lower.length; i++) {
                    Mat process = new Mat();
                    ArrayList<MatOfPoint> contours = new ArrayList<>();
                    Rect maxRect = new Rect();
                    int minAreaThreshold = 5000;
                    Imgproc.cvtColor(input, process, Imgproc.COLOR_RGB2YCrCb);
                    Core.inRange(process, lower[i], upper[i], process);
                    Imgproc.morphologyEx(process, process, Imgproc.MORPH_OPEN, new Mat());
                    Imgproc.morphologyEx(process, process, Imgproc.MORPH_CLOSE, new Mat());
                    Imgproc.GaussianBlur(process, process, new Size(15, 5), 0);
                    Imgproc.findContours(process, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                    Imgproc.drawContours(output, contours, -1, new Scalar(0, 0, 255));
                    for (MatOfPoint contour : contours) {
                        Point[] contourArr = contour.toArray();
                        Rect boundingRect = Imgproc.boundingRect(new MatOfPoint2f(contourArr));
                        if (contourArr.length >= 15 && boundingRect.area() > minAreaThreshold && boundingRect.area() > maxRect.area()) {
                            maxRect = boundingRect.clone();
                        }
                    }
                    if (maxRect.area() > maxColor[0]) {
                        maxColor = new int[] {(int) maxRect.area(), i};
                    }
                    Imgproc.rectangle(output, maxRect, new Scalar(255, 0, 0));
                    Imgproc.putText(output, "Color "+ (i + 1) + " Area: " + maxRect.area(), new Point(10, 10 + 10 * i), 0, 1, new Scalar(0, 0, 0), 2);
                    process.release();
                }
                caseDetected = maxColor[1] + 1;
                if (caseDetected == 0) {
                    Imgproc.putText(output, "Signal not detected", new Point(10, 630), 0, 1, new Scalar(0, 0, 0), 2);
                } else {
                    Imgproc.putText(output, "Case: " + caseDetected, new Point(10, 630), 0, 1, new Scalar(0, 0, 0), 2);
                }
                output.release();
                return input;
            }
        };
    }
    public void initialize() {
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);}
            @Override
            public void onError(int errorCode) {};
        });
    }
    public void end() {
        camera.stopStreaming();
    }
    public int caseDetected() {
        return caseDetected;
    }
}