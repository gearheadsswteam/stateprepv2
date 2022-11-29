package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.classes.SignalDetector;
public abstract class AbstractAutonomous extends LinearOpMode {
    public Robot robot;
    SignalDetector detector;
    int runCase = 1;
    int caseDetected = 1;
    int caseDetectionLength = 0;
    public int side = 0;
    public Pose2d initPose = new Pose2d(0, 0, 0);
    public Pose2d finalPose = new Pose2d(0, 0, 0);
    ElapsedTime clock = new ElapsedTime();
    double time;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, armIn, wristIn, gripperHold);
        detector = new SignalDetector(hardwareMap);
        detector.init();
        robot.drive.setPoseEstimate(initPose);
        initialize();
        while (!isStarted() && !isStopRequested()) {
            time = clock.seconds();
            if (detector.getCaseDetected() == caseDetected) {
                caseDetectionLength++;
            } else if (detector.getCaseDetected() > 0) {
                caseDetected = detector.getCaseDetected();
                caseDetectionLength = 1;
            }
            if (caseDetectionLength >= signalMinCount) {
                runCase = caseDetected;
            }
            robot.update(time);
            telemetry.addData("Case Detected", caseDetected);
            telemetry.update();
        }
        detector.end();
        redMultiplier = side;
        lastPose = finalPose;
        run();
        lastPose = robot.drive.getPoseEstimate();
    }
    public abstract void initialize();
    public abstract void run();
}
