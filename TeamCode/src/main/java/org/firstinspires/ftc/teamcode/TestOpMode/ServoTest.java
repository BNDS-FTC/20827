package org.firstinspires.ftc.teamcode.TestOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "servo test")
@Config
public class ServoTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static boolean read_only = false;
    public static double servo0_pos = 0.5;
    public static String servo_name = "liftServo";
    private Servo servo0;

    @Override
    public void runOpMode() {

        servo0 = hardwareMap.get(Servo.class, servo_name);
//        servo1 = hardwareMap.get(Servo.class,"servo1");

        waitForStart();
        while (opModeIsActive()) {
            if (!read_only) {
                servo0.setPosition(servo0_pos);
//                servo1.setPosition(servo1_pos-val);
            }
        }
    }
}
