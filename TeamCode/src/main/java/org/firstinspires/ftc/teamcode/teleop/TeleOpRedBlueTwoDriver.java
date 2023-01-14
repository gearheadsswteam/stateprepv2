package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.INTAKE_POWER_AUTO;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.INTAKE_POWER_TELEOP;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armIn;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armRest;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.backArmProfile1;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.backWristProfile1;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardArmProfile1;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardArmProfile2;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardWristProfile1;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.forwardWristProfile2;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.gripperHold;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.gripperRelease;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.holderDetectionThreshold;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.holderMinCount;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftGroundClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftHighClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftHighFar;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftLowClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftMedClose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.odoUp;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.rollerDown;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.rollerRetract;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.rollerUp;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.side;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.wristIn;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.wristRest;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.classes.ProfileChain;
import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.classes.ValueStorage;

@TeleOp(name = "TeleOpRedBlueTwoDriver")
public class TeleOpRedBlueTwoDriver extends LinearOpMode {

    public static final double INTAKE_REVERSE_POWER = -0.75;
    Robot robot = new Robot();
    int state = 0;
    int holderDetectionCount = 0;
    double initialHeading = ValueStorage.lastPose.getHeading() - side * PI / 2;
    double robotHeading;
    double moveAngle;
    double moveMagnitude;
    double turn;
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
        robot.init(hardwareMap, armRest, wristRest);
        robot.gripper.setPosition(gripperRelease);
        robot.retract.setPosition(odoUp);
        robot.roller.setPosition(rollerDown);
        while (!isStarted() && !isStopRequested()) {
            robot.update(clock.seconds());
        }
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
            if (gamepad2.ps) {
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
                        if (!stateDir && time > robot.armTime()) {
                            robot.gripper.setPosition(gripperRelease);
                        }
                    } else {
                        if (gamepad1.right_trigger < 0.3) {
                            robot.setIntakePowers(INTAKE_POWER_TELEOP, INTAKE_POWER_TELEOP);
                        } else if (gamepad1.right_trigger < 0.7) {
                            robot.setIntakePowers(0, 0);
                        } else {
                            robot.setIntakePowers(INTAKE_REVERSE_POWER, INTAKE_REVERSE_POWER);
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
                            robot.armProfile = forwardArmProfile1(time);
                            robot.wristProfile = forwardWristProfile1(time);
                            stateTime = robot.armTime();
                            robot.gripper.setPosition(gripperHold);
                            robot.roller.setPosition(rollerRetract);
                        } else if (lbPressed) {
                            state = 4;
                            stateDir = false;
                            stateTime = time;
                            robot.setIntakePowers(0, 0);
                            robot.roller.setPosition(rollerRetract);
                        }
                    }
                    break;
                case 1:
                    if (time > stateTime) {
                        robot.setIntakePowers(0, 0);
                        if (aPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftLowClose[0], 0);
                            robot.extendArmProfile(time, liftLowClose[1], 0);
                            robot.extendWristProfile(time, liftLowClose[2], 0);
                            stateTime = robot.restTime();
                        } else if (bPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftMedClose[0], 0);
                            robot.extendArmProfile(time, liftMedClose[1], 0);
                            robot.extendWristProfile(time, liftMedClose[2], 0);
                            stateTime = robot.restTime();
                        } else if (gamepad1.right_trigger > 0.2 && yPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftHighFar[0], 0);
                            robot.extendArmProfile(time, liftHighFar[1], 0);
                            robot.extendWristProfile(time, liftHighFar[2], 0);
                            stateTime = robot.restTime();
                        } else if (yPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftHighClose[0], 0);
                            robot.extendArmProfile(time, liftHighClose[1], 0);
                            robot.extendWristProfile(time, liftHighClose[2], 0);
                            stateTime = robot.restTime();
                        } else if (xPressed) {
                            state = 2;
                            robot.extendLiftProfile(time, liftGroundClose[0], 0);
                            robot.extendArmProfile(time, liftGroundClose[1], 0);
                            robot.extendWristProfile(time, liftGroundClose[2], 0);
                            stateTime = robot.restTime();
                        }
                        if (lbPressed) {
                            state = 0;
                            stateDir = false;
                            robot.armProfile = backArmProfile1(time);
                            robot.wristProfile = backWristProfile1(time);
                            stateTime = robot.armTime() + 0.25;
                            robot.roller.setPosition(rollerDown);
                        }
                    } else if (time > robot.armProfile.getTi()) {
                        robot.setIntakePowers(-0.5, -0.5);
                    }/*else if (robot.armProfile.getX(time) < armIn) {
                        if (aPressed) {
                            double upTime = robot.armTime();
                            state = 2;
                            robot.extendLiftProfile(time, liftLowClose[0], 0);
                            robot.armProfile = new ProfileChain(robot.armProfile)
                                    .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, upTime, armUp, 0, liftLowClose[1], 0));
                            robot.wristProfile = new ProfileChain(robot.wristProfile)
                                    .add(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, upTime, wristUp, 0, liftLowClose[2], 0));
                            stateTime = robot.restTime();
                        } else if (bPressed) {
                            double upTime = robot.armTime();
                            state = 2;
                            robot.extendLiftProfile(time, liftMedClose[0], 0);
                            robot.armProfile = new ProfileChain(robot.armProfile)
                                    .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, upTime, armUp, 0, liftMedClose[1], 0));
                            robot.wristProfile = new ProfileChain(robot.wristProfile)
                                    .add(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, upTime, wristUp, 0, liftMedClose[2], 0));
                            stateTime = robot.restTime();
                        } else if (gamepad1.right_trigger > 0.2 && yPressed) {
                            double upTime = robot.armTime();
                            state = 2;
                            robot.extendLiftProfile(time, liftHighFar[0], 0);
                            robot.armProfile = new ProfileChain(robot.armProfile)
                                    .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, upTime, armUp, 0, liftHighFar[1], 0));
                            robot.wristProfile = new ProfileChain(robot.wristProfile)
                                    .add(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, upTime, wristUp, 0, liftHighFar[2], 0));
                            stateTime = robot.restTime();
                        } else if (yPressed) {
                         +   double upTime = robot.armTime();
                            state = 2;
                            robot.extendLiftProfile(time, liftHighClose[0], 0);
                            robot.armProfile = new ProfileChain(robot.armProfile)
                                    .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, upTime, armUp, 0, liftHighClose[1], 0));
                            robot.wristProfile = new ProfileChain(robot.wristProfile)
                                    .add(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, upTime, wristUp, 0, liftHighClose[2], 0));
                            stateTime = robot.restTime();
                        } else if (xPressed) {
                            double upTime = robot.armTime();
                            state = 2;
                            robot.extendLiftProfile(time, liftGroundClose[0], 0);
                            robot.armProfile = new ProfileChain(robot.armProfile)
                                    .add(new TrapezoidalProfile(armMaxVel, armMaxAccel, upTime, armUp, 0, liftGroundClose[1], 0));
                            robot.wristProfile = new ProfileChain(robot.wristProfile)
                                    .add(new TrapezoidalProfile(wristMaxVel, wristMaxAccel, upTime, wristUp, 0, liftGroundClose[2], 0));
                            stateTime = robot.restTime();
                        }
                    } */
                    break;
                case 2:
                    if (time < stateTime) {} else if (aPressed) {
                        robot.extendLiftProfile(time, liftLowClose[0], 0);
                        robot.extendArmProfile(time, liftLowClose[1], 0);
                        robot.extendWristProfile(time, liftLowClose[2], 0);
                        stateTime = robot.restTime();
                    } else if (bPressed) {
                        robot.extendLiftProfile(time, liftMedClose[0], 0);
                        robot.extendArmProfile(time, liftMedClose[1], 0);
                        robot.extendWristProfile(time, liftMedClose[2], 0);
                        stateTime = robot.restTime();
                    } else if (gamepad1.right_trigger > 0.2 && yPressed) {
                        robot.extendLiftProfile(time, liftHighFar[0], 0);
                        robot.extendArmProfile(time, liftHighFar[1], 0);
                        robot.extendWristProfile(time, liftHighFar[2], 0);
                        stateTime = robot.restTime();
                    } else if (yPressed) {
                        robot.extendLiftProfile(time, liftHighClose[0], 0);
                        robot.extendArmProfile(time, liftHighClose[1], 0);
                        robot.extendWristProfile(time, liftHighClose[2], 0);
                        stateTime = robot.restTime();
                    } else if (xPressed) {
                        robot.extendLiftProfile(time, liftGroundClose[0], 0);
                        robot.extendArmProfile(time, liftGroundClose[1], 0);
                        robot.extendWristProfile(time, liftGroundClose[2], 0);
                        stateTime = robot.restTime();
                    } else if (rbPressed) {
                        state = 3;
                        stateDir = true;
                        stateTime = 0.5;
                        robot.gripper.setPosition(gripperRelease);
                    }
                    break;
                case 3:
                    if (time < stateTime) {
                    } else if (rbPressed) {
                        state = 4;
                        robot.extendLiftProfile(time, 0, 0);
                        robot.extendArmProfile(time, armIn, 0);
                        robot.extendWristProfile(time, wristIn, 0);
                        stateTime = robot.restTime();
                    } else if (lbPressed) {
                        state = 2;
                        stateDir = false;
                        stateTime = 0.5;
                        robot.gripper.setPosition(gripperHold);
                    }
                    break;
                case 4:
                    if (time < stateTime) {
                        if (rbPressed && stateDir) {
                            state = 0;
                            double downTime = robot.restTime();
                            robot.armProfile = new ProfileChain(robot.armProfile)
                                    .add(forwardArmProfile2(downTime));
                            robot.wristProfile = new ProfileChain(robot.wristProfile)
                                    .add(forwardWristProfile2(downTime));
                            stateTime = robot.armTime();
                            robot.roller.setPosition(rollerDown);
                        }
                    } else if (rbPressed) {
                        state = 0;
                        if (stateDir) {
                            robot.armProfile = forwardArmProfile2(time);
                            robot.wristProfile = forwardWristProfile2(time);
                            stateTime = robot.armTime();
                        } else {
                            stateTime = time;
                        }
                        stateDir = true;
                        robot.roller.setPosition(rollerDown);
                    }
                    break;
            }
            if (gamepad1.dpad_up && (state == 2 || state == 3) && time > stateTime) {
                //robot.extendLiftProfile(time, adjustUp(robot.liftProfile.getX(time)), 0);
            } else if (gamepad1.dpad_down && (state == 2 || state == 3) && time > stateTime) {
                //robot.extendLiftProfile(time, adjustDown(robot.liftProfile.getX(time)), 0);
            }
            robot.update(time);
            robotHeading = robot.getHeading() + initialHeading;
            moveAngle = atan2(-gamepad2.left_stick_x, -gamepad2.left_stick_y) - robotHeading;
            moveMagnitude = abs(pow(gamepad2.left_stick_x, 3)) + abs(pow(gamepad2.left_stick_y, 3));
            if (moveMagnitude < 0.01) {
                moveMagnitude = 0;
            }
            turn = pow(gamepad2.right_stick_x, 3);
            if(gamepad2.right_trigger < 0.1) {
                robot.setDrivePowers(moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn,
                        moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn,
                        moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn,
                        moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
            }else {
                robot.setDrivePowers((moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn)/4,
                        (moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn)/4,
                        (moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn)/4,
                        (moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn)/4);
            }
        }
    }
}