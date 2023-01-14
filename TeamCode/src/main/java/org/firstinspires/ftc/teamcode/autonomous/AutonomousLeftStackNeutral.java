package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import static java.lang.Math.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.ValueStorage;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "LeftStackNeutral", group = "Left")
public class AutonomousLeftStackNeutral extends AbstractAutonomous {
    public static final int INTAKE_POWER = 1;
    Pose2d dropPose = new Pose2d(46, 9, -2.65);
    Pose2d[] parkPose = {new Pose2d(59, 11, PI), new Pose2d(35, 11, PI), new Pose2d(11, 11, PI)};
    Pose2d stackPose = new Pose2d(61, 11, PI);
    Pose2d knockPose = new Pose2d(51, 11, PI);
    Pose2d backPose = new Pose2d(55, 11, PI);
    Pose2d intakePose = new Pose2d(63, 11, PI);
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
                .splineTo(new Vector2d(35, 40), -PI / 2)
                .lineTo(new Vector2d(35, 25))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToSplineHeading(dropPose, 1.15)
                .waitSeconds(0.3)
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
                .splineTo(stackPose.vec(), stackPose.getHeading() + PI)
                .lineTo(knockPose.vec())
                .addDisplacementMarker(() -> {
                    usingSensor = true;
                    robot.roller.setPosition(rollerDown);
                    robot.setIntakePowers(INTAKE_POWER, INTAKE_POWER);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(intakePose.vec())
                .addTemporalMarker(1, 0, () -> {
                    intakeTrajDone = true;
                })
                .build();

        //Stack to the drop point
        traj3 = robot.drive.trajectorySequenceBuilder(backPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, -1.3, () -> {
                    robot.extendLiftProfile(time, liftHighClose[0], 0);
                })
                .addTemporalMarker(1, 0, () -> {
                    endDropTraj = true;
                    dropTrajDone = true;
                    dropTrajTime = time;
                    cycles++;
                })
                .build();

        //Drop point to stack for the 2nd cone
        traj4 = robot.drive.trajectorySequenceBuilder(dropPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                .splineTo(backPose.vec(), backPose.getHeading() + PI)
                .addDisplacementMarker(() -> {
                    usingSensor = true;
                    robot.roller.setPosition(rollerDown);
                    robot.setIntakePowers(INTAKE_POWER, INTAKE_POWER);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(2, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(intakePose.vec())
                .addTemporalMarker(1, 0, () -> {
                    intakeTrajDone = true;
                })
                .build();

        //Drop poin to park
        traj5 = new TrajectorySequence[] {
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[0].vec(), 0)
                        .addTemporalMarker(1, 0, () -> {
                            traj5Done = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[0].vec(), 0)
                        .lineTo(parkPose[1].vec())
                        .addTemporalMarker(1, 0, () -> {
                            traj5Done = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[0].vec(), 0)
                        .lineTo(parkPose[2].vec())
                        .addTemporalMarker(1, 0, () -> {
                            traj5Done = true;
                        })
                        .build()

        };
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

        while (opModeIsActive() && !isStopRequested() && (!traj5Done || time > retractTime)) {
            time = clock.seconds();


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
            if (dropTrajDone && time - dropTrajTime > 0.5) {
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
                } else if (cycles < 2) {
                    robot.drive.followTrajectorySequenceAsync(traj4);
                    robot.extendLiftProfile(time, 0, 0);
                    robot.extendArmProfile(time, armIn, 0);
                    robot.extendWristProfile(time, wristIn, 0);
                    retractTime = robot.restTime();
                    retractDone = true;
                    dropTrajDone = false;
                } else {//Two cycles for three cones done...go to park
                    robot.drive.followTrajectorySequenceAsync(traj5[runCase - 1]);
                    robot.setIntakePowers(0, 0);
                    robot.extendLiftProfile(time, 0, 0);
                    robot.extendArmProfile(time, armIn, 0);
                    robot.extendWristProfile(time, wristIn, 0);
                    retractTime = robot.restTime();
                    retractDone = true;
                    dropTrajDone = false;
                }
            }

            //**
            // Cone is inside
            //Go to drop position
            if (intakeTrajDone || (usingSensor && robot.holder.getDistance(DistanceUnit.INCH) < holderDetectionThreshold)) {//cone is inside
                TrajectorySequence tempTraj = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .lineTo(backPose.vec())
                        .addTemporalMarker(0, 0.5, () -> {
                            robot.setIntakePowers(-0.5, -0.5);
                        })
                        .addTemporalMarker(1, 0, () -> {
                            robot.drive.followTrajectorySequenceAsync(traj3);
                        })
                        .build();
                robot.drive.followTrajectorySequenceAsync(tempTraj);
                robot.armProfile = forwardArmProfile1(time);
                //.addExtendTrapezoidal(armMaxVel, armMaxAccel, liftHighClose[1], 0);
                robot.wristProfile = forwardWristProfile1(time);
                //.addExtendTrapezoidal(armMaxVel, armMaxAccel, liftHighClose[1], 0);
                robot.setIntakePowers(0, 0);
                robot.roller.setPosition(rollerRetract);
                robot.gripper.setPosition(gripperHold);
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
        return new Pose2d(32, 60, -PI / 2);
    }

}

