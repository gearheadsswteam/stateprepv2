package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.classes.SignalDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "AutonomousBlueRightParkShared", group = "BlueRight")
public class AutonomousBlueRightParkShared extends LinearOpMode {
    public Robot robot;
    SignalDetector detector;
    int runCase = 1;
    int caseDetected = 1;
    int caseDetectionLength = 0;
    public int side = 0;
    public Pose2d initPose = new Pose2d(-35, 60, -PI/2);
    public Pose2d dropPose = new Pose2d(-30, 24, 1.2);
    public Pose2d[] parkPose = new Pose2d[] {new Pose2d(-12, 12, -PI/2), new Pose2d(-35, 12, -PI/2), new Pose2d(-57, 12, -PI/2)};
    TrajectorySequence traj1;
    TrajectorySequence[] traj2;
    boolean startTraj1 = true;
    boolean startTraj2 = false;
    boolean endTraj2 = false;
    boolean done = false;
    ElapsedTime clock = new ElapsedTime();
    double time;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, armIn, wristIn, gripperHold);
        detector = new SignalDetector(hardwareMap);
        detector.init();
        robot.drive.setPoseEstimate(initPose);
        traj1 = robot.drive.trajectorySequenceBuilder(initPose)
                .lineTo(new Vector2d (-35, 30))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .build();
        traj2 = new TrajectorySequence[] {
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[1].vec(), parkPose[1].getHeading())
                        .lineTo(parkPose[0].vec())
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[1].vec(), parkPose[1].getHeading())
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[1].vec(), parkPose[1].getHeading())
                        .lineTo(parkPose[2].vec())
                        .build(),
        };
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
        //lastPose = finalPose;
        //run();
        lastPose = robot.drive.getPoseEstimate();
    }
}
