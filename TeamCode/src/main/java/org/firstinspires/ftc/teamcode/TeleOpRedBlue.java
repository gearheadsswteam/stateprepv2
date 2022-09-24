package org.firstinspires.ftc.teamcode;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@TeleOp(name = "TeleOpRedBlue", group = "TeleOp")
public class TeleOpRedBlue extends LinearOpMode {
    DcMotorEx fr;
    DcMotorEx fl;
    DcMotorEx br;
    DcMotorEx bl;
    DcMotorEx arm1;
    DcMotorEx arm2;
    Servo claw;
    BNO055IMU gyro;
    double initialHeading = ValueStorage.lastPose.getHeading() - redMultiplier * PI / 2;
    double robotHeading;
    double moveAngle;
    double moveMagnitude;
    double turn;
    double l1 = 264;
    double l2 = 312;
    double theta1;
    double theta2;
    double dx;
    double dy;
    double dTheta1;
    double dTheta2;
    int arm1Pos = 0;
    int arm2Pos = 0;
    int state = 0;
    int armState = 0;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean aReleased = true;
    boolean bReleased = true;
    boolean xPressed = false;
    boolean xReleased = false;
    boolean yPressed = false;
    boolean yReleased = false;
    boolean rbPressed = false;
    boolean rbReleased = false;
    ElapsedTime stateTimer = new ElapsedTime();
    @Override
    public void runOpMode() {
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        claw = hardwareMap.get(Servo.class, "claw");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fl.setDirection(Direction.REVERSE);
        bl.setDirection(Direction.REVERSE);
        arm1.setDirection(Direction.REVERSE);
        arm1.setTargetPosition(0);
        arm2.setTargetPosition(0);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        arm1.setPower(0.5);
        arm2.setPower(0.5);
        claw.setPosition(clawOpen);
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
            if (gamepad1.dpad_up) {
                dy = 1;
            } else if (gamepad1.dpad_down) {
                dy = -1;
            } else {
                dy = 0;
            }
            if (gamepad1.dpad_right) {
                dx = 1;
            } else if (gamepad1.dpad_left) {
                dx = -1;
            } else {
                dx = 0;
            }
            switch (state) {
                case 0:
                    if (stateTimer.milliseconds() < stateTimes[0][armState]) {
                        arm1Pos = 0;
                        arm2Pos = 0;
                        dx = 0;
                        dy = 0;
                    } else if (rbPressed) {
                        state = 1;
                        stateTimer.reset();
                    }
                    break;
                case 1:
                    if (stateTimer.milliseconds() < stateTimes[1][armState]) {
                        claw.setPosition(clawClosed);
                    } else if (aPressed) {
                        state = 2;
                        armState = 0;
                        stateTimer.reset();
                    } else if (bPressed) {
                        state = 2;
                        armState = 1;
                        stateTimer.reset();
                    } else if (yPressed) {
                        state = 2;
                        armState = 2;
                        stateTimer.reset();
                    } else if (xPressed) {
                        state = 2;
                        armState = 3;
                        stateTimer.reset();
                    }
                    break;
                case 2:
                    if (stateTimer.milliseconds() < stateTimes[2][armState]) {
                        arm1Pos = arm1States[armState];
                        arm2Pos = arm2States[armState];
                        dx = 0;
                        dy = 0;
                    } else if (rbPressed) {
                        state = 3;
                        stateTimer.reset();
                    } else if (aPressed) {
                        armState = 0;
                        arm1Pos = arm1States[armState];
                        arm2Pos = arm2States[armState];
                    } else if (bPressed) {
                        armState = 1;
                        arm1Pos = arm1States[armState];
                        arm2Pos = arm2States[armState];
                    } else if (yPressed) {
                        armState = 2;
                        arm1Pos = arm1States[armState];
                        arm2Pos = arm2States[armState];
                    } else if (xPressed) {
                        armState = 3;
                        arm1Pos = arm1States[armState];
                        arm2Pos = arm2States[armState];
                    }
                    break;
                case 3:
                    if (stateTimer.milliseconds() < stateTimes[3][armState]) {
                        claw.setPosition(clawOpen);
                    } else if (rbPressed) {
                        state = 0;
                        stateTimer.reset();
                    }
                    break;
            }
            theta1 = initTheta1 + arm1Pos / countsPerRad1;
            theta2 = initTheta2 + arm2Pos / countsPerRad2;
            dTheta1 = (cos(theta2) * dx + sin(theta2) * dy) / (l1 * sin(theta1 - theta2));
            dTheta2 = (cos(theta1) * dx + sin(theta1) * dy) / (l2 * sin(theta2 - theta1));
            if ((theta1 + dTheta1 - initTheta1) / countsPerRad1 > arm1Min && (theta1 + dTheta1 - initTheta1) / countsPerRad1 < arm1Max && (theta2 + dTheta2 - initTheta2) / countsPerRad2 > arm2Min + arm1Pos && (theta2 + dTheta2 - initTheta2) / countsPerRad2 < arm2Max + arm1Pos) {
                arm1Pos += dTheta1 * countsPerRad1;
                arm2Pos += dTheta2 * countsPerRad2;
            }
            arm1.setTargetPosition(arm1Pos);
            arm2.setTargetPosition(arm2Pos);
            robotHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + initialHeading;
            moveAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) - robotHeading;
            moveMagnitude = abs(pow(gamepad1.left_stick_x, 3)) + abs(pow(gamepad1.left_stick_y, 3));
            turn = pow(gamepad1.right_stick_x, 3);
            fr.setPower(moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn);
            fl.setPower(moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn);
            br.setPower(moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
            bl.setPower(moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn);
        }
    }
}
