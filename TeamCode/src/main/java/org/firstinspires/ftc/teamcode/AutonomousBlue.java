package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous (name = "AutonomousBlue")
public class AutonomousBlue extends LinearOpMode {
    SampleMecanumDrive drive;
    SignalDetector detector;
    int runCase = 1;
    int caseDetected = 1;
    int caseDetectionLength = 0;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        detector = new SignalDetector(hardwareMap);
        detector.initialize();
        while (!isStarted() && !isStopRequested()) {
            if (detector.getCaseDetected() == caseDetected) {
                caseDetectionLength++;
            } else if (detector.getCaseDetected() > 0) {
                caseDetected = detector.getCaseDetected();
                caseDetectionLength = 1;
            }
            if (caseDetectionLength >= caseDetectionThreshold) {
                runCase = caseDetected;
            }
            telemetry.addData("Case Detected", caseDetected);
            telemetry.update();
        }
        detector.end();
        ValueStorage.redMultiplier = -1;
        ValueStorage.lastPose = new Pose2d(0, 0, 0);
    }
}