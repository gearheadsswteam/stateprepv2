package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.MotionProfile;
import org.firstinspires.ftc.teamcode.classes.ProfileChain;
import org.firstinspires.ftc.teamcode.classes.TrapezoidalProfile;

@TeleOp(name = "ServoTest", group = "TeleOp")
public class ServoTest extends LinearOpMode {
    Servo armL;
    Servo armR;
    Servo wristL;
    Servo wristR;
    MotionProfile armProfile = new TrapezoidalProfile(armMaxVel, armMaxAccel, 0, armRest, 0, armRest, 0);
    MotionProfile wristProfile = new TrapezoidalProfile(armMaxVel, armMaxAccel, 0, wristRest, 0, wristRest, 0);
    ElapsedTime clock = new ElapsedTime();
    Telemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    double armPos = 0.5;
    double wristPos = 0.5;
    double time = 0;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean aReleased = true;
    boolean bReleased = true;
    boolean xPressed = false;
    boolean xReleased = false;
    boolean yPressed = false;
    boolean yReleased = false;
    boolean up = false;
    @Override
    public void runOpMode() {
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            time = clock.seconds();
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
            if (aPressed && time > max(armProfile.getTf(), wristProfile.getTf())) {
                if (up) {
                    armProfile = armProfile.extendTrapezoidal(armMaxVel, armMaxAccel, time, armIn, 0);
                    double inTime = armProfile.getTf();
                    armProfile = new ProfileChain(armProfile).add(forwardArmProfile2(inTime));
                    wristProfile = new ProfileChain(wristProfile.extendTrapezoidal(wristMaxVel, wristMaxAccel, time, wristIn, 0)).add(forwardWristProfile2(inTime));
                    up = false;
                } else {
                    armProfile = forwardArmProfile1(time);
                    wristProfile = forwardWristProfile1(time);
                    up = true;
                }
            }
            armL.setPosition(armProfile.getX(time));
            armR.setPosition(armOffset - armProfile.getX(time));
            wristL.setPosition(wristProfile.getX(time));
            wristR.setPosition(wristOffset - wristProfile.getX(time));
            multipleTelemetry.addData("Arm Position", armProfile.getX(time));
            multipleTelemetry.addData("Wrist Position", wristProfile.getX(time));
            multipleTelemetry.update();
        }
    }
}