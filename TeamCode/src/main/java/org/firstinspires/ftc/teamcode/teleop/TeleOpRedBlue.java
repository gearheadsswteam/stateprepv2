package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.classes.ValueStorage;
@TeleOp(name = "TeleOpRedBlue")
public class TeleOpRedBlue extends LinearOpMode {
    Robot robot = new Robot();
    int state = 0;
    int holderDetectionCount = 0;
    double initialHeading = ValueStorage.lastPose.getHeading() - redMultiplier * PI / 2;
    double robotHeading;
    double moveAngle;
    double moveMagnitude;
    double turn;
    double switchTime = 0;
    double stateTime = 0;
    double time;
    boolean stateDir = true;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean aReleased = true;
    boolean bReleased = true;
    boolean xPressed = false;
    boolean xReleased = false;
    boolean yPressed = false;
    boolean yReleased = false;
    boolean lbPressed = false;
    boolean lbReleased = false;
    boolean rbPressed = false;
    boolean rbReleased = false;
    ElapsedTime clock = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, armRest, wristRest, gripperRelease);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                aPressed = aReleased;
                aReleased = false;
            } else {
                aPressed = false;
                aReleased = true;
            }
            if (gamepad1.b) {
                bPressed = bReleased;
                bReleased = false;
            } else {
                bPressed = false;
                bReleased = true;
            }
            if (gamepad1.x) {
                xPressed = xReleased;
                xReleased = false;
            } else {
                xPressed = false;
                xReleased = true;
            }
            if (gamepad1.y) {
                yPressed = yReleased;
                yReleased = false;
            } else {
                yPressed = false;
                yReleased = true;
            }
            if (gamepad1.left_bumper) {
                lbPressed = lbReleased;
                lbReleased = false;
            } else {
                lbPressed = false;
                lbReleased = true;
            }
            if (gamepad1.right_bumper) {
                rbPressed = rbReleased;
                rbReleased = false;
            } else {
                rbPressed = false;
                rbReleased = true;
            }
            if (gamepad1.ps) {
                initialHeading -= robotHeading;
            }
            if (robot.holder.getDistance(DistanceUnit.INCH) < holderDetectionThreshold) {
                holderDetectionCount++;
            } else {
                holderDetectionCount = 0;
            }
            time = clock.seconds();
            switch (state) {
                case 0:
                    if (time < stateTime) {
                        if (!stateDir && time > robot.armProfile.getT()) {
                            robot.gripper.setPosition(gripperRelease);
                        }
                    } else {

                        if (gamepad1.right_trigger < 0.3) {
                            robot.setIntakePowers(1, 1);
                        } else if (gamepad1.right_trigger < 0.7) {
                            robot.setIntakePowers(0, 0);
                        } else {
                            robot.setIntakePowers(-0.5, -0.5);
                        }
                        if (gamepad1.left_trigger < 0.3) {
                            robot.roller.setPosition(rollerDown);
                        } else if (gamepad1.left_trigger < 0.7) {
                            robot.roller.setPosition((1.75 - 2.5 * gamepad1.left_trigger) * rollerDown + (2.5 * gamepad1.left_trigger - 0.75) * rollerUp);
                        } else {
                            robot.roller.setPosition(rollerUp);
                        }
                        if (rbPressed || holderDetectionCount >= holderMinCount) {
                            state = 1;
                            stateDir = true;
                            robot.setIntakePowers(-0.5, -0.5);
                            robot.armProfile = forwardArmProfile1(time);
                            robot.wristProfile = forwardWristProfile1(time);
                            stateTime = robot.armProfile.getT();
                            switchTime = time;
                            robot.gripper.setPosition(gripperHold);
                            robot.roller.setPosition(rollerRetract);
                        } else if (lbPressed) {
                            state = 4;
                            stateDir = false;
                            stateTime = time;
                            switchTime = time;
                            robot.setIntakePowers(0, 0);
                            robot.roller.setPosition(rollerRetract);
                        }
                    }
                    break;
                case 1:
                    if (time < stateTime) {} else {
                        robot.setIntakePowers(0, 0);
                        if (aPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftLowClose[0], 0);
                            robot.extendArmProfile(time, liftLowClose[1], 0);
                            robot.extendWristProfile(time, liftLowClose[2], 0);
                            stateTime = robot.restTime();
                            switchTime = time;
                        } else if (bPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftMedClose[0], 0);
                            robot.extendArmProfile(time, liftMedClose[1], 0);
                            robot.extendWristProfile(time, liftMedClose[2], 0);
                            stateTime = robot.restTime();
                            switchTime = time;
                        } else if (gamepad1.right_trigger > 0.2 && yPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftHighFar[0], 0);
                            robot.extendArmProfile(time, liftHighFar[1], 0);
                            robot.extendWristProfile(time, liftHighFar[2], 0);
                            stateTime = robot.restTime();
                            switchTime = time;
                        } else if (yPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftHighClose[0], 0);
                            robot.extendArmProfile(time, liftHighClose[1], 0);
                            robot.extendWristProfile(time, liftHighClose[2], 0);
                            stateTime = robot.restTime();
                            switchTime = time;
                        } else if (xPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftGroundClose[0], 0);
                            robot.extendArmProfile(time, liftGroundClose[1], 0);
                            robot.extendWristProfile(time, liftGroundClose[2], 0);
                            stateTime = robot.restTime();
                            switchTime = time;
                        } else if (lbPressed) {
                            state = 0;
                            stateDir = false;
                            robot.armProfile = backArmProfile1(time);
                            robot.wristProfile = backWristProfile1(time);
                            stateTime = robot.armProfile.getT() + 0.25;
                            switchTime = time;
                            robot.roller.setPosition(rollerDown);
                        }
                    }
                    break;
                case 2:
                    if (time < stateTime) {} else if (aPressed) {
                        robot.extendLiftProfile(time, liftLowClose[0], 0);
                        robot.extendArmProfile(time, liftLowClose[1], 0);
                        robot.extendWristProfile(time, liftLowClose[2], 0);
                        stateTime = robot.restTime();
                        switchTime = time;
                    } else if (bPressed) {
                        robot.extendLiftProfile(time, liftMedClose[0], 0);
                        robot.extendArmProfile(time, liftMedClose[1], 0);
                        robot.extendWristProfile(time, liftMedClose[2], 0);
                        stateTime = robot.restTime();
                        switchTime = time;
                    } else if (gamepad1.right_trigger > 0.2 && yPressed) {
                        robot.extendLiftProfile(time, liftHighFar[0], 0);
                        robot.extendArmProfile(time, liftHighFar[1], 0);
                        robot.extendWristProfile(time, liftHighFar[2], 0);
                        stateTime = robot.restTime();
                        switchTime = time;
                    } else if (yPressed) {
                        robot.extendLiftProfile(time, liftHighClose[0], 0);
                        robot.extendArmProfile(time, liftHighClose[1], 0);
                        robot.extendWristProfile(time, liftHighClose[2], 0);
                        stateTime = robot.restTime();
                        switchTime = time;
                    } else if (xPressed) {
                        robot.extendLiftProfile(time, liftGroundClose[0], 0);
                        robot.extendArmProfile(time, liftGroundClose[1], 0);
                        robot.extendWristProfile(time, liftGroundClose[2], 0);
                        stateTime = robot.restTime();
                        switchTime = time;
                    } else if (rbPressed) {
                        state = 3;
                        stateDir = true;
                        stateTime = 0.5;
                        switchTime = time;
                        robot.gripper.setPosition(gripperRelease);
                    }
                    break;
                case 3:
                    if (time < stateTime) {} else if (rbPressed) {
                        state = 4;
                        robot.extendLiftProfile(time, 0, 0);
                        robot.extendArmProfile(time, armIn, 0);
                        robot.extendWristProfile(time, wristIn, 0);
                        stateTime = robot.restTime();
                        switchTime = time;
                    } else if (lbPressed) {
                        state = 2;
                        stateDir = false;
                        stateTime = 0.5;
                        switchTime = time;
                        robot.gripper.setPosition(gripperHold);
                    }
                    break;
                case 4:
                    if (time < stateTime) {} else if (rbPressed) {
                        state = 0;
                        if (stateDir) {
                            robot.armProfile = forwardArmProfile2(time);
                            robot.wristProfile = forwardWristProfile2(time);
                            stateTime = robot.armProfile.getT() + 0.25;
                        } else {
                            stateTime = time;
                        }
                        switchTime = time;
                        stateDir = true;
                        robot.roller.setPosition(rollerDown);
                    }
                    break;
            }
            robot.update(time);
            robotHeading = robot.getHeading() + initialHeading;
            moveAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) - robotHeading;
            moveMagnitude = min(abs(pow(gamepad1.left_stick_x, 3)) + abs(pow(gamepad1.left_stick_y, 3)), 0.01);
            turn = pow(gamepad1.right_stick_x, 3);
            robot.setDrivePowers(moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn, 
                    moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn, 
                    moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn, 
                    moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
        }
    }
}