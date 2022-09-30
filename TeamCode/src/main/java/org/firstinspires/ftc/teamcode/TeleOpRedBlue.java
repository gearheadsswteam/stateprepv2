package org.firstinspires.ftc.teamcode;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    BNO055IMU gyro;
    double initialHeading = ValueStorage.lastPose.getHeading() - redMultiplier * PI / 2;
    double robotHeading;
    double moveAngle;
    double moveMagnitude;
    double turn;
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
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fl.setDirection(Direction.REVERSE);
        bl.setDirection(Direction.REVERSE);
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
