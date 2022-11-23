package org.firstinspires.ftc.teamcode;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "ServoTest", group = "TeleOp")
public class ServoTest extends LinearOpMode {
    Servo armL;
    Servo armR;
    Servo wristL;
    Servo wristR;
    double armPos = armRest;
    double wristPos = wristRest;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean aReleased = true;
    boolean bReleased = true;
    boolean xPressed = false;
    boolean xReleased = false;
    boolean yPressed = false;
    boolean yReleased = false;
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
                if (aPressed) {
                    wristPos = min(1, wristPos + 0.1);
                } else if (xPressed) {
                    wristPos = min(1, wristPos + 0.01);
                } else if (bPressed) {
                    wristPos = max(0, wristPos - 0.1);
                } else if (yPressed) {
                    wristPos = max(0, wristPos - 0.01);
                }
            } else {
                if (aPressed) {
                    armPos = min(1, armPos + 0.1);
                } else if (xPressed) {
                    armPos = min(1, armPos + 0.01);
                } else if (bPressed) {
                    armPos = max(0, armPos - 0.1);
                } else if (yPressed) {
                    armPos = max(0, armPos - 0.01);
                }
            }
            armL.setPosition(armPos);
            armR.setPosition(armOffset - armPos);
            wristL.setPosition(wristPos);
            wristR.setPosition(wristOffset - wristPos);
            telemetry.addData("Arm Position", armPos);
            telemetry.addData("Wrist Position", wristPos);
            telemetry.update();
        }
    }
}