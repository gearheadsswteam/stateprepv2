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
import org.firstinspires.ftc.teamcode.util.PidfController;
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
    double adjust = 0;
    double adjustIncrement = 10;
    double liftPos = 0;
    double armPos = armRest;
    double wristPos = wristRest;
    double stateTime = 0;
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
    ElapsedTime stateTimer = new ElapsedTime();
    PidfController liftPidf = new PidfController(liftKp, liftKi, liftKd) {
        @Override
        public double kf(double input) {
            return liftKf (input);
        }
    };
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
            switch (state) {
                case 0:
                    if (stateDir && stateTimer.seconds() < stateTime) {
                        armPos = forwardSafeArmPos2(stateTimer.seconds())[0];
                        wristPos = forwardSafeArmPos2(stateTimer.seconds())[1];
                        gripper.setPosition(forwardSafeArmPos2(stateTimer.seconds())[2]);
                    } else if (!stateDir && stateTimer.seconds() < stateTime) {
                        armPos = backSafeArmPos1(stateTimer.seconds())[0];
                        wristPos = backSafeArmPos1(stateTimer.seconds())[1];
                        gripper.setPosition(backSafeArmPos1(stateTimer.seconds())[2]);
                    } else {
                        intakeL.setPower(1);
                        intakeR.setPower(1);
                        armPos = armRest;
                        wristPos = wristRest;
                        gripper.setPosition(gripperRelease);
                        if (rbPressed || holderDetectionCount >= holderConfidence) {
                            state = 1;
                            stateDir = true;
                            stateTime = forwardStateTimes(liftPos, armL.getPosition(), wristL.getPosition())[1];
                            stateTimer.reset();
                            intakeL.setPower(-0.5);
                            intakeR.setPower(-0.5);
                            roller.setPosition(rollerRetract);
                        } else if (lbPressed) {
                            state = 4;
                            stateDir = false;
                            stateTime = backStateTimes[4];
                            stateTimer.reset();
                            intakeL.setPower(0);
                            intakeR.setPower(0);
                            roller.setPosition(rollerRetract);
                        }
                    }
                    break;
                case 1:
                    if (stateTimer.seconds() < stateTime) {
                        armPos = forwardSafeArmPos1(stateTimer.seconds())[0];
                        wristPos = forwardSafeArmPos1(stateTimer.seconds())[1];
                        gripper.setPosition(forwardSafeArmPos1(stateTimer.seconds())[2]);
                    } else {
                        intakeL.setPower(0);
                        intakeR.setPower(0);
                        armPos = armUp;
                        wristPos = wristUp;
                        gripper.setPosition(gripperHold);
                        if (aPressed) {
                            state = 3;
                            stateDir = true;
                            stateTime = forwardStateTimes(liftLow[0], liftLow[1], liftLow[2])[2];
                            stateTimer.reset();
                            liftPos = liftLow[0];
                            armPos = liftLow[1];
                            wristPos = liftLow[2];
                        } else if (bPressed) {
                            state = 3;
                            stateDir = true;
                            stateTime = forwardStateTimes(liftMed[0], liftMed[1], liftMed[2])[2];
                            liftPos = liftMed[0];
                            armPos = liftMed[1];
                            wristPos = liftMed[2];
                        } else if (yPressed) {
                            state = 3;
                            stateDir = true;
                            stateTime = forwardStateTimes(liftHigh[0], liftHigh[1], liftHigh[2])[2];
                            liftPos = liftHigh[0];
                            armPos = liftHigh[1];
                            wristPos = liftHigh[2];
                        } else if (xPressed) {
                            state = 2;
                            stateDir = true;
                            stateTime = forwardStateTimes(liftGround[0], liftGround[1], liftGround[2])[2];
                            liftPos = liftGround[0];
                            armPos = liftGround[1];
                            wristPos = liftGround[2];
                        } else if (lbPressed) {
                            state = 0;
                            stateDir = false;
                            stateTime = backStateTimes[0];
                            stateTimer.reset();
                        }
                    }
                    break;
                case 2:
                    if (stateTimer.seconds() < stateTime) {} else if (aPressed) {
                        liftPos = liftLow[0];
                        armPos = liftLow[1];
                        wristPos = liftLow[2];
                    } else if (bPressed) {
                        liftPos = liftMed[0];
                        armPos = liftMed[1];
                        wristPos = liftMed[2];
                    } else if (yPressed) {
                        liftPos = liftHigh[0];
                        armPos = liftHigh[1];
                        wristPos = liftHigh[2];
                    } else if (xPressed) {
                        liftPos = liftGround[0];
                        armPos = liftGround[1];
                        wristPos = liftGround[2];
                    } else if (rbPressed) {
                        state = 3;
                        stateDir = true;
                        stateTime = forwardStateTimes(liftPos, armPos, wristPos)[3];
                        stateTimer.reset();
                        gripper.setPosition(gripperRelease);
                    }
                    break;
                case 3:
                    if (stateTimer.seconds() < stateTime) {} else if (rbPressed) {
                        state = 4;
                        stateDir = true;
                        stateTime = forwardStateTimes(liftL.getCurrentPosition(), armPos, wristPos)[4];
                        stateTimer.reset();
                        liftPos = 0;
                        armPos = armUp;
                        wristPos = wristUp;
                    } else if (lbPressed) {
                        state = 2;
                        stateDir = false;
                        stateTime = backStateTimes[2];
                        stateTimer.reset();
                        gripper.setPosition(gripperHold);
                    }
                    break;
                case 4:
                    if (stateTimer.seconds() < stateTime) {} else if (rbPressed) {
                        state = 0;
                        if (stateDir) {
                            stateTime = forwardStateTimes(liftPos, armPos, wristPos)[0];
                        } else {
                            stateTime = 0;
                        }
                        stateDir = true;
                        stateTimer.reset();
                        roller.setPosition(rollerDown);
                    }
                    break;
            }
            armL.setPosition(armPos);
            armR.setPosition(armOffset - armPos);
            wristL.setPosition(wristPos);
            wristR.setPosition(wristOffset - wristPos);
            liftL.setPower(liftPidf.get());
            liftR.setPower(liftPidf.get());
            liftPidf.set(liftPos);
            liftPidf.update(liftL.getCurrentPosition());
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
