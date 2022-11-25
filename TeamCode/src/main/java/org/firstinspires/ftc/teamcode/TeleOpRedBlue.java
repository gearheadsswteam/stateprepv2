package org.firstinspires.ftc.teamcode;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.PidfController;
import org.firstinspires.ftc.teamcode.util.ProfileChain;
import org.firstinspires.ftc.teamcode.util.TrapezoidalProfile;

import java.util.ArrayList;

@TeleOp(name = "TeleOpRedBlue", group = "TeleOp")
public class TeleOpRedBlue extends LinearOpMode {
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotorEx intakeL;
    DcMotorEx intakeR;
    DcMotorEx liftL;
    DcMotorEx liftR;
    Servo gripper;
    Servo retract;
    Servo roller;
    Servo angleL;
    Servo angleR;
    Servo armL;
    Servo armR;
    Servo wristL;
    Servo wristR;
    BNO055IMU gyro;
    RevColorSensorV3 holder;
    int state = 0;
    int holderDetectionCount = 0;
    double initialHeading = ValueStorage.lastPose.getHeading() - redMultiplier * PI / 2;
    double robotHeading;
    double moveAngle;
    double moveMagnitude;
    double turn;
    double armPos = armRest;
    double wristPos = wristRest;
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
    PidfController liftPidf = new PidfController(liftKp, liftKi, liftKd) {
        @Override
        public double kf(double input) {
            return liftKf (input);
        }
    };
    TrapezoidalProfile liftProfile = new TrapezoidalProfile(liftMaxVel, liftMaxAccel, 0, 0, 0, 0, 0);
    MotionProfile armProfile = new TrapezoidalProfile(armMaxVel, armMaxAccel, 0, armRest, 0, armRest, 0);
    MotionProfile wristProfile = new TrapezoidalProfile(wristMaxVel, wristMaxAccel, 0, wristRest, 0, wristRest, 0);
    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        intakeL = hardwareMap.get(DcMotorEx.class, "intakeL");
        intakeR = hardwareMap.get(DcMotorEx.class, "intakeR");
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");
        gripper = hardwareMap.get(Servo.class, "gripper");
        //retract = hardwareMap.get(Servo.class, "retract");
        roller = hardwareMap.get(Servo.class, "roller");
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        //holder = hardwareMap.get(RevColorSensorV3.class, "holder");
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        intakeL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        liftL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fl.setDirection(Direction.REVERSE);
        bl.setDirection(Direction.REVERSE);
        intakeR.setDirection(Direction.REVERSE);
        liftR.setDirection(Direction.REVERSE);
        gripper.setPosition(gripperRelease);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro.initialize(parameters);
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
            /* if (holder.getDistance(DistanceUnit.INCH) < holderDetectionThreshold) {
                holderDetectionCount++;
            } else {
                holderDetectionCount = 0;
            } */
            time = clock.seconds();
            switch (state) {
                case 0:
                    if (time - switchTime < stateTime) {
                        if (time - switchTime > armProfile.getT()) {
                            gripper.setPosition(gripperRelease);
                        }
                    } else {
                        if (gamepad1.right_trigger > 0.3) {
                            intakeL.setPower(0);
                            intakeR.setPower(0);
                        } else if (gamepad1.right_trigger > 0.7) {
                            intakeL.setPower(-0.5);
                            intakeR.setPower(-0.5);
                        } else {
                            intakeL.setPower(1);
                            intakeR.setPower(1);
                        }
                        if (gamepad1.left_trigger > 0.7) {
                            roller.setPosition(rollerUp);
                        } else if (gamepad1.left_trigger > 0.3) {
                            roller.setPosition((1.75 - 2.5 * gamepad1.left_trigger) * rollerDown + (2.5 * gamepad1.left_trigger - 0.75) * rollerUp);
                        } else {
                            roller.setPosition(rollerDown);
                        }
                        if (rbPressed || holderDetectionCount >= holderConfidence) {
                            state = 1;
                            stateDir = true;
                            intakeL.setPower(-0.5);
                            intakeR.setPower(-0.5);
                            armProfile = forwardSafeArmPos1;
                            wristProfile = forwardSafeWristPos1;
                            stateTime = time + armProfile.getT();
                            switchTime = time;
                            gripper.setPosition(gripperHold);
                            roller.setPosition(rollerRetract);
                        } else if (lbPressed) {
                            state = 4;
                            stateDir = false;
                            stateTime = 500;
                            switchTime = time;
                            intakeL.setPower(0);
                            intakeR.setPower(0);
                            roller.setPosition(rollerRetract);
                        }
                    }
                    break;
                case 1:
                    if (time - switchTime < stateTime) {} else {
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        if (aPressed) {
                            state = 2;
                            stateDir = true;
                            liftProfile = liftProfile.extendTrapezoidal(time, liftLow[0], 0);
                            armProfile = new ProfileChain().add(armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, liftLow[1], 0));
                            wristProfile = new ProfileChain().add(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, liftLow[2], 0));
                            stateTime = max(max(liftProfile.getT(), armProfile.getT()), wristProfile.getT());
                            switchTime = time;
                        } else if (bPressed) {
                            state = 2;
                            stateDir = true;
                            liftProfile = liftProfile.extendTrapezoidal(time, liftMed[0], 0);
                            armProfile = new ProfileChain().add(armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, liftMed[1], 0));
                            wristProfile = new ProfileChain().add(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, liftMed[2], 0));
                            stateTime = max(max(liftProfile.getT(), armProfile.getT()), wristProfile.getT());
                            switchTime = time;
                        } else if (yPressed) {
                            state = 2;
                            stateDir = true;
                            liftProfile = liftProfile.extendTrapezoidal(time, liftHigh[0], 0);
                            armProfile = new ProfileChain().add(armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, liftHigh[1], 0));
                            wristProfile = new ProfileChain().add(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, liftHigh[2], 0));
                            stateTime = max(max(liftProfile.getT(), armProfile.getT()), wristProfile.getT());
                            switchTime = time;
                        } else if (xPressed) {
                            state = 2;
                            stateDir = true;
                            liftProfile = liftProfile.extendTrapezoidal(time, liftGround[0], 0);
                            armProfile = new ProfileChain().add(armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, liftGround[1], 0));
                            wristProfile = new ProfileChain().add(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, liftGround[2], 0));
                            stateTime = max(max(liftProfile.getT(), armProfile.getT()), wristProfile.getT());
                            switchTime = time;
                        } else if (lbPressed) {
                            state = 0;
                            stateDir = false;
                            armProfile = backSafeArmPos1;
                            wristProfile = backSafeWristPos1;
                            stateTime = time + armProfile.getT() + 0.25;
                            switchTime = time;
                            roller.setPosition(rollerDown);
                        }
                    }
                    break;
                case 2:
                    if (time - switchTime < stateTime) {} else if (aPressed) {
                        liftProfile = liftProfile.extendTrapezoidal(time, liftLow[0], 0);
                        armProfile = new ProfileChain().add(armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, liftLow[1], 0));
                        wristProfile = new ProfileChain().add(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, liftLow[2], 0));
                    } else if (bPressed) {
                        liftProfile = liftProfile.extendTrapezoidal(time, liftMed[0], 0);
                        armProfile = new ProfileChain().add(armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, liftMed[1], 0));
                        wristProfile = new ProfileChain().add(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, liftMed[2], 0));
                    } else if (yPressed) {
                        liftProfile = liftProfile.extendTrapezoidal(time, liftHigh[0], 0);
                        armProfile = new ProfileChain().add(armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, liftHigh[1], 0));
                        wristProfile = new ProfileChain().add(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, liftHigh[2], 0));
                    } else if (xPressed) {
                        liftProfile = liftProfile.extendTrapezoidal(time, liftGround[0], 0);
                        armProfile = new ProfileChain().add(armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, liftGround[1], 0));
                        wristProfile = new ProfileChain().add(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, liftGround[2], 0));
                    } else if (rbPressed) {
                        state = 3;
                        stateDir = true;
                        stateTime = 0.5;
                        switchTime = time;
                        gripper.setPosition(gripperRelease);
                    }
                    break;
                case 3:
                    if (time - switchTime < stateTime) {} else if (rbPressed) {
                        state = 4;
                        stateDir = true;
                        liftProfile = liftProfile.extendTrapezoidal(time, 0, 0);
                        armProfile = new ProfileChain().add(armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, armIn, 0));
                        wristProfile = new ProfileChain().add(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, wristIn, 0));
                        stateTime = max(max(liftProfile.getT(), armProfile.getT()), wristProfile.getT());
                        switchTime = time;
                    } else if (lbPressed) {
                        state = 2;
                        stateDir = false;
                        stateTime = 0.5;
                        switchTime = time;
                        gripper.setPosition(gripperHold);
                    }
                    break;
                case 4:
                    if (time - switchTime < stateTime) {} else if (rbPressed) {
                        state = 0;
                        if (stateDir) {
                            armProfile = forwardSafeArmPos2;
                            wristProfile = forwardSafeWristPos2;
                            stateTime = time + armProfile.getT() + 0.25;
                        } else {
                            stateTime = 0;
                        }
                        switchTime = time;
                        stateDir = true;
                        roller.setPosition(rollerDown);
                    }
                    break;
            }
            liftPidf.set(liftProfile.getX(time));
            liftPidf.update(liftL.getCurrentPosition());
            liftL.setPower(liftPidf.get());
            liftR.setPower(liftPidf.get());
            armL.setPosition(armPos);
            armR.setPosition(armOffset - armPos);
            wristL.setPosition(wristPos);
            wristR.setPosition(wristOffset - wristPos);
            robotHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + initialHeading;
            moveAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) - robotHeading;
            moveMagnitude = abs(pow(gamepad1.left_stick_x, 3)) + abs(pow(gamepad1.left_stick_y, 3));
            turn = pow(gamepad1.right_stick_x, 3);
            fl.setPower(moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn);
            fr.setPower(moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn);
            bl.setPower(moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn);
            br.setPower(moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
        }
    }
}
