package org.firstinspires.ftc.teamcode;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "ServoTest", group = "TeleOp")
public class ServoTest extends LinearOpMode {
    Servo armL;
    Servo armR;
    Servo wristL;
    Servo wristR;
    boolean aPressed = false;
    boolean aReleased = true;
    boolean bPressed = false;
    boolean bReleased = true;
    boolean xPressed = false;
    boolean xReleased = true;
    boolean yPressed = false;
    boolean yReleased = true;
    boolean lbPressed = false;
    boolean lbReleased = true;
    boolean started = false;
    double time;
    ElapsedTime clock = new ElapsedTime();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() {
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                aPressed = aReleased;
                aReleased = false;
            } else {
                aPressed = false;
                aReleased = true;
            }
            if (gamepad1.b) {;
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
            time = clock.seconds();
            if (started) {
                armL.setPosition(forwardSafeArmPos1.getX(time));
                armR.setPosition(armOffset - forwardSafeArmPos1.getX(time));
                wristL.setPosition(forwardSafeWristPos1.getX(time));
                wristR.setPosition(wristOffset - forwardSafeWristPos1.getX(time));
            } else {
                armL.setPosition(armRest);
                armR.setPosition(armOffset - armRest);
                wristL.setPosition(wristRest);
                wristR.setPosition(wristOffset - wristRest);
                if (aPressed) {
                    started = true;
                    clock.reset();
                }
            }
            multipleTelemetry.addData("Arm Position", armL.getPosition());
            multipleTelemetry.addData("Wrist Position", wristL.getPosition());
            multipleTelemetry.update();
        }
    }
}