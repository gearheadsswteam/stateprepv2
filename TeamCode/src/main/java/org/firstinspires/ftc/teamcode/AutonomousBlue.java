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
        detector = new SignalDetector(hardwareMap, lower, upper);
        while (!isStarted() && !isStopRequested()) {
            if (detector.caseDetected() == caseDetected) {
                caseDetectionLength++;
            } else {
                caseDetected = detector.caseDetected();
                caseDetectionLength = 1;
            }
            if (caseDetectionLength >= caseDetectionThreshold) {
                runCase = caseDetected;
            }
        }
        detector.end();
        ValueStorage.redMultiplier = -1;
        ValueStorage.lastPose = new Pose2d(0, 0, 0);
    }
}