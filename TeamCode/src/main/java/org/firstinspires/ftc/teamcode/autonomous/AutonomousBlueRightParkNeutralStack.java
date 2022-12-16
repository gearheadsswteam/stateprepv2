package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import static java.lang.Math.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    Pose2d dropPose = new Pose2d(-45, 9.5, -0.45);
    Pose2d[] parkPose = new Pose2d[]{new Pose2d(-11, 34, -PI / 2), new Pose2d(-35, 34, -PI / 2), new Pose2d(-59, 34, -PI / 2)};
    Pose2d knockPose = new Pose2d(-60, 11, 0);
    Pose2d backPose = new Pose2d(-50, 11, 0);
    Pose2d intakePose = new Pose2d(-64, 11, 0);
    ElapsedTime clock = new ElapsedTime();
    double time = 0;
    double dropTrajTime = 1000;
    double retractTime = 1000;
    double doneTime = 1000;

    boolean endDropTraj = false;
    boolean dropTrajDone = false;
    boolean traj5Done = false;
    boolean intakeTrajDone = false;

    boolean retractDone = true;
    boolean usingSensor = false;

    int cycles = 0;

    TrajectorySequence traj1; //From start to drop point
    TrajectorySequence traj2; //From drop to stack
    TrajectorySequence traj3; //From stack to drop
    TrajectorySequence traj4; //From drop to stack
    TrajectorySequence[] traj5; //From drop to park

    @Override
    public void initialize() {
        //Start to drop point
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .splineTo(new Vector2d(-35, 40), -PI / 2)
                .lineTo(new Vector2d(-35, 25))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToSplineHeading(dropPose, 2)
                .waitSeconds(0.5)
                .resetConstraints()
                .addTemporalMarker(1, -1, () -> {
                    robot.extendLiftProfile(time, liftHighClose[0], 0);
                })
                .addTemporalMarker(1, 0, () -> {
                    endDropTraj = true;
                    dropTrajDone = true;
                    dropTrajTime = time;
                }).build();

        //Drop point to stack
        traj2 = robot.drive.trajectorySequenceBuilder(dropPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                .splineTo(knockPose.vec(), knockPose.getHeading() + PI)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(backPose.vec())
                .addDisplacementMarker(() -> {
                    usingSensor = true;
                    robot.roller.setPosition(rollerDown);
                    robot.setIntakePowers(1, 1);
                })
                .lineTo(intakePose.vec())
                .addTemporalMarker(1, 0, () -> {
                    intakeTrajDone = true;
                })
                .build();

        //Stack to the drop point
        traj3 = robot.drive.trajectorySequenceBuilder(backPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, -1.3, () -> {
                    robot.extendLiftProfile(time, liftHighClose[0], 0);
                })
                .waitSeconds(1.3)
                .addTemporalMarker(1, 0, () -> {
                    endDropTraj = true;
                    dropTrajDone = true;
                    dropTrajTime = time;
                    cycles++;
                })
                .build();

        //Drop point to stack for the 2nd cone
        traj4 = robot.drive.trajectorySequenceBuilder(dropPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                .splineTo(backPose.vec(), backPose.getHeading() + PI)
                .addDisplacementMarker(() -> {
                    usingSensor = true;
                    robot.roller.setPosition(rollerDown);
                    robot.setIntakePowers(1, 1);
                })
                .lineTo(intakePose.vec())
                .build();

        //Drop poin to park
        traj5 = null;
    }


    @Override
    public void run() {
        clock.reset();

        //Set up the arm init position
        robot.armProfile = autonomousArmProfile(0);
                //.addExtendTrapezoidal(armMaxVel, armMaxAccel, liftHighClose[1], 0);
        robot.wristProfile = autonomousWristProfile(0);
                //.addExtendTrapezoidal(wristMaxVel, wristMaxAccel, liftHighClose[2],0);

        robot.drive.followTrajectorySequenceAsync(traj1);

        //while (opModeIsActive() && !isStopRequested() && (!traj1Done || time < doneTime)) {
        while (opModeIsActive() && !isStopRequested()) {
            time = clock.seconds();

            // Cone detection inside
            if (robot.holder.getDistance(DistanceUnit.INCH) < holderDetectionThreshold) {
                holderDetectionCount++;
            } else {
                holderDetectionCount = 0;
            }

            //Starting the lift in Traj 1 & Traj 3...as you are going to the drop point

            //You are at drop position
            if (endDropTraj) {
                robot.gripper.setPosition(gripperRelease);
                endDropTraj = false;
            }


            //Execute Traj 2 from Drop position to stack
            //Drop the stack
            //GO back until cone is intaken
            //Give 1 second for cone to drop
            if (dropTrajDone && time - dropTrajTime > 1) {
                if (cycles == 0) {
                    robot.drive.followTrajectorySequenceAsync(traj2);
                    //Retract the lift
                    robot.extendLiftProfile(time, 0, 0);
                    robot.extendArmProfile(time, armIn, 0);
                    robot.extendWristProfile(time, wristIn, 0);
                    robot.roller.setPosition(rollerUp);
                    robot.setIntakePowers(0, 0.5);
                    retractTime = robot.restTime();
                    dropTrajDone = false;
                } else if (cycles < 3) {
                    robot.drive.followTrajectorySequenceAsync(traj4);
                    robot.extendLiftProfile(time, 0, 0);
                    robot.extendArmProfile(time, armIn, 0);
                    robot.extendWristProfile(time, wristIn, 0);
                    retractTime = robot.restTime();
                    retractDone = true;
                    dropTrajDone = false;
                } else {//Three cycles for three cones done...go to park
                        robot.drive.followTrajectorySequenceAsync(traj5[runCase - 1]);
                        robot.extendLiftProfile(time, 0, 0);
                        robot.extendArmProfile(time, armIn, 0);
                        robot.extendWristProfile(time, wristIn, 0);
                        retractTime = robot.restTime();
                }
            }

            //**
            // Cone is inside
            //Go to drop position
            if (intakeTrajDone || (usingSensor && holderDetectionCount > holderMinCount)) {//cone is inside
                TrajectorySequence tempTraj = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .lineTo(intakePose.vec())
                        .addTemporalMarker(1, 0, () -> {
                            robot.drive.followTrajectorySequenceAsync(traj3);
                        })
                        .build();
                robot.drive.followTrajectorySequenceAsync(tempTraj);
                robot.armProfile = forwardArmProfile1(time);
                        //.addExtendTrapezoidal(armMaxVel, armMaxAccel, liftHighClose[1], 0);
                robot.wristProfile = forwardWristProfile1(time);
                        //.addExtendTrapezoidal(armMaxVel, armMaxAccel, liftHighClose[1], 0);
                robot.roller.setPosition(rollerRetract);
                robot.gripper.setPosition(gripperHold);
                robot.setIntakePowers(-0.5, -0.5);
                usingSensor = false;
                intakeTrajDone = false;
                //Cone is inside
            }


            //You are at drop position

            /**
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
             **/
            if (retractDone && time > retractTime) {
                robot.armProfile = forwardArmProfile2(time);
                robot.wristProfile = forwardWristProfile2(time);
                doneTime = robot.armTime();
                retractDone = false;
            }


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
        if (dropTrajDone && time - dropTrajTime > 1) {
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
