package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "AutonomousBlueRightParkSpecific", group = "BlueRight")
public class AutonomousBlueRightParkSpecific extends AbstractAutonomous {
    Pose2d dropPose = new Pose2d(-20, 33, -0.45);
    Pose2d[] parkPose = new Pose2d[] {new Pose2d(-11, 34, 0), new Pose2d(-35, 34, 0), new Pose2d(-59, 34, 0)};
    TrajectorySequence traj1;
    TrajectorySequence[] traj2;
    ElapsedTime clock = new ElapsedTime();
    double time = 0;
    double traj1Time = 1000;
    double retractTime = 1000;
    double doneTime = 1000;
    boolean startLift = false;
    boolean endTraj1 = false;
    boolean traj1Done = false;
    boolean traj2Done = false;
    boolean retractDone = true;
    @Override
    public void initialize() {
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(-35, 48), -PI / 2)
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .resetConstraints()
                .addTemporalMarker(1, -1.6, () -> {
                    startLift = true;
                })
                .addTemporalMarker(1, 0, () -> {
                    endTraj1 = true;
                    traj1Done = true;
                    traj1Time = clock.seconds();
                }).build();
        traj2 = new TrajectorySequence[] {
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[1].vec(), parkPose[1].getHeading() + PI)
                        .lineTo(parkPose[0].vec())
                        .addTemporalMarker(1, 0, () -> {
                            traj2Done = true;
                        }).build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[1].vec(), parkPose[1].getHeading() + PI)
                        .addTemporalMarker(1, 0, () -> {
                            traj2Done = true;
                        }).build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[1].vec(), parkPose[1].getHeading() + PI)
                        .lineTo(parkPose[2].vec())
                        .addTemporalMarker(1, 0, () -> {
                            traj2Done = true;
                        }).build(),
        };
    }
    @Override
    public void run() {
        clock.reset();
        robot.armProfile = autonomousArmProfile(0);
        robot.wristProfile = autonomousWristProfile(0);
        robot.drive.followTrajectorySequenceAsync(traj1);
        while(opModeIsActive() && !isStopRequested() && (!traj2Done || time < doneTime)) {
            time = clock.seconds();
            if (startLift) {
                robot.extendLiftProfile(time, liftHighClose[0], 0);
                robot.extendArmProfile(time, liftHighClose[1], 0);
                robot.extendWristProfile(time, liftHighClose[2], 0);
                startLift = false;
            }
            if (endTraj1) {
                robot.gripper.setPosition(gripperRelease);
                endTraj1 = false;
            }
            if (traj1Done && time - traj1Time > 1) {
                robot.drive.followTrajectorySequenceAsync(traj2[runCase - 1]);
                robot.extendLiftProfile(time, 0, 0);
                robot.extendArmProfile(time, armIn, 0);
                robot.extendWristProfile(time, wristIn, 0);
                retractTime = robot.restTime();
                traj1Done = false;
            }
            if (retractDone && time > retractTime) {
                robot.armProfile = forwardArmProfile2(time);
                robot.wristProfile = forwardWristProfile2(time);
                doneTime = robot.armProfile.getTf();
                retractDone = false;
            }
            robot.drive.update();
            robot.update(time);
        }
    }
    @Override
    public int side() {
        return sides.BLUE;
    }
    @Override
    public Pose2d initPose() {
        return new Pose2d(-32, 60, -PI / 2);
    }
}
