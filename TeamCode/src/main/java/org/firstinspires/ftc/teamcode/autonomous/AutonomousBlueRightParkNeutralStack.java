package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armIn;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.autonomousArmProfile;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.autonomousWristProfile;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardArmProfile2;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardWristProfile2;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.gripperRelease;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.holderDetectionThreshold;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.holderMinCount;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftHighClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftHighFar;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.rollerUp;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.wristIn;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.ValueStorage;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Right-Stack", group = "Right")
public class AutonomousBlueRightParkNeutralStack extends AbstractAutonomous {

    int holderDetectionCount = 0;
    Pose2d dropPose = new Pose2d(-45, 8, -0.4);
    Pose2d[] parkPose = new Pose2d[]{new Pose2d(-11, 34, -PI / 2), new Pose2d(-35, 34, -PI / 2), new Pose2d(-59, 34, -PI / 2)};

    ElapsedTime clock = new ElapsedTime();
    double time = 0;
    double traj1Time = 1000;
    double traj3Time = 1000;
    double retractTime = 1000;
    double doneTime = 1000;

    boolean startLift = false;
    boolean endTraj1 = false;
    boolean endTraj3 = false;
    boolean traj1Done = false;
    boolean traj2Done = false;
    boolean traj3Done = false;
    boolean traj4Done = false;
    boolean traj5Done = false;

    boolean retractDone = true;
    boolean usingSensor = false;

    int cycles = 0;

    TrajectorySequence traj1; //From start to drop point
    TrajectorySequence traj2; //From drop to stack
    TrajectorySequence traj3; //From stack to drop
    TrajectorySequence traj4; //From drop to stack
    TrajectorySequence traj5; //From drop to park

    @Override
    public void initialize() {
        //Start to drop point
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(-35, 40), -PI / 2)
                .lineTo(new Vector2d(-35, 25))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToSplineHeading(dropPose, 2)
                .resetConstraints()
                .addTemporalMarker(1, -1.0, () -> {
                    startLift = true;
                })
                .addTemporalMarker(1, 0, () -> {
                    endTraj1 = true;
                    traj1Done = true;
                    traj1Time = clock.seconds();
                }).build();

        //Drop point to stack
        traj2 = null; //

        //Stack to the drop point
        traj3 = null;

        //Drop point to stack for the 2nd cone
        traj4 = null;

        //Drop poin to park
        traj5 = null;
    }


    @Override
    public void run() {
        clock.reset();

        //Set up the arm init position
        robot.armProfile = autonomousArmProfile(0);
        robot.wristProfile = autonomousWristProfile(0);

        robot.drive.followTrajectorySequenceAsync(traj1);

        //while (opModeIsActive() && !isStopRequested() && (!traj1Done || time < doneTime)) {
        while (opModeIsActive() && !isStopRequested() && (!traj1Done)) {
            time = clock.seconds();

            // Cone detection inside
            if (robot.holder.getDistance(DistanceUnit.INCH) < holderDetectionThreshold) {
                holderDetectionCount++;
            } else {
                holderDetectionCount = 0;
            }

            //Starting the lift in Traj 1 & Traj 3...as you are going to the drop point
            if (startLift) {
                robot.extendLiftProfile(time, liftHighClose[0], 0);
                robot.extendArmProfile(time, liftHighClose[1], 0);
                robot.extendWristProfile(time, liftHighClose[2], 0);
                startLift = false;
            }

            //You are at drop position
            if (endTraj1) {
                robot.gripper.setPosition(gripperRelease);
                endTraj1 = false;
            }

            /**
            //Execute Traj 2 from Drop position to stack
            //Drop the stack
            //GO back until cone is intaken
            //Give 1 second for cone to drop
            if (traj1Done && time - traj1Time > 1) {
                robot.drive.followTrajectorySequenceAsync(traj2);
                traj1Done = false;
            }

            // Cone is inside
            //Go to drop position
            if (usingSensor && holderDetectionCount > holderMinCount) {//cone is inside
                robot.drive.followTrajectorySequenceAsync(traj3);
                usingSensor = false;
                //Cone is inside
            }

            //You are at drop position
            if (endTraj3) {
                robot.gripper.setPosition(gripperRelease);
                endTraj3 = false;
            }

            if (traj3Done && time - traj3Time > 1) {
                //three cones
                if (cycles < 3) {
                    robot.drive.followTrajectorySequenceAsync(traj4);
                } else {//Three cycles for three cones done...go to park
                    robot.drive.followTrajectorySequenceAsync(traj5[runCase - 1]);
                    robot.extendLiftProfile(time, 0, 0);
                    robot.extendArmProfile(time, armIn, 0);
                    robot.extendWristProfile(time, wristIn, 0);
                    retractTime = robot.restTime();
                }
                traj3Done = false;
            }

            if (retractDone && time > retractTime) {
                robot.armProfile = forwardArmProfile2(time);
                robot.wristProfile = forwardWristProfile2(time);
                doneTime = robot.armTime();
                retractDone = false;
            }

             **/

            sleep(3000);

            robot.drive.update();
            robot.update(time);
        }
    }

    @Override
    public int side() {
        return ValueStorage.sides.BLUE;
    }

    @Override
    public Pose2d initPose() {
        return new Pose2d(-32, 60, -PI / 2);
    }

    public void executeStackPickup() {
        if (traj1Done && time - traj1Time > 1) {
            robot.drive.followTrajectorySequenceAsync(traj3);
            robot.setIntakePowers(1, 1);
            robot.roller.setPosition(rollerUp);
            //Wait till stack is down

            //Move robot forward slowly till stack is on the floor

            //Move Robot back slowly till first cone is consumed

            robot.extendLiftProfile(time, liftHighFar[0], 0);
            robot.extendArmProfile(time, liftHighFar[1], 0);
            robot.extendWristProfile(time, liftHighFar[2], 0);
        }
    }


}
