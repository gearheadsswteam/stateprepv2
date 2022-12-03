package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.PidfController;
import org.firstinspires.ftc.teamcode.classes.TrapezoidalProfile;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Disabled
@TeleOp (name = "PidfTest", group = "TeleOp")
public class PidfTest extends LinearOpMode {
    public static double KP = liftKp;
    public static double KI = liftKi;
    public static double KD = liftKd;
    public static double A_MAX = liftMaxAccel;
    public static double V_MAX = liftMaxVel;
    double kfTest(double input) {
        return liftKf(input);
    }
    DcMotorEx test1;
    DcMotorEx test2;
    PidfController lift = new PidfController(KP, KI, KD) {
        public double kf(double input) {
            return kfTest (input);
        }
    };
    TrapezoidalProfile currentProfile = new TrapezoidalProfile(V_MAX, A_MAX,0, 0, 0, 0, 0);
    ElapsedTime clock = new ElapsedTime();
    Telemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    double[][] lastPos = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
    double pos;
    double time ;
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
        test1 = hardwareMap.get(DcMotorEx.class, "liftL");
        test2 = hardwareMap.get(DcMotorEx.class, "liftR");
        test2.setDirection(DcMotorSimple.Direction.REVERSE);
        test1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        test1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        test1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            time = clock.seconds();
            pos = test1.getCurrentPosition();
            if (aPressed) {
                currentProfile = currentProfile.extendTrapezoidal(V_MAX, A_MAX, time, min(1100, currentProfile.getX(time) + 500), 0);
            } else if (yPressed) {
                currentProfile = currentProfile.extendTrapezoidal(V_MAX, A_MAX, time, min(1100, currentProfile.getX(time) + 100), 0);
            } else if (bPressed) {
                currentProfile = currentProfile.extendTrapezoidal(V_MAX, A_MAX, time, max(0, currentProfile.getX(time) - 500), 0);
            } else if (xPressed) {
                currentProfile = currentProfile.extendTrapezoidal(V_MAX, A_MAX, time, max(0, currentProfile.getX(time) - 100), 0);
            }
            multipleTelemetry.addData("Position", pos);
            multipleTelemetry.addData("Velocity", (pos - lastPos[8][0]) / (time - lastPos[8][1]));
            multipleTelemetry.addData("Set Point", currentProfile.getX(time));
            multipleTelemetry.addData("Set Velocity", currentProfile.getV(time));
            multipleTelemetry.addData("Power", lift.get());
            multipleTelemetry.update();
            lift.update(pos);
            lift.set(currentProfile.getX(time));
            lift.setConstants(KP, KI, KD);
            test1.setPower(lift.get());
            test2.setPower(lift.get());
            for (int i = lastPos.length - 2; i >= 0; i--) {
                lastPos[i + 1][0] = lastPos[i][0];
                lastPos[i + 1][1] = lastPos[i][1];
            }
            lastPos[0] = new double[] {pos, time};
        }
    }
}