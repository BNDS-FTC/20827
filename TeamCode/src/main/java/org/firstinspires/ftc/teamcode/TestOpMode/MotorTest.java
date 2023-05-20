package org.firstinspires.ftc.teamcode.TestOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "motor test")
@Config
public class MotorTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int encoder_position = 0;
    public static double max_power = 0.6;
    public static boolean read_only = true;
    public static boolean reverse = false;
    public static boolean reset = false;
    public static boolean set_power_mode_or_set_position_mode = false;
    public static String motor_name = "lift1";

    @Override
    public void runOpMode() {
        DcMotorEx motor0 = hardwareMap.get(DcMotorEx.class, motor_name);
        waitForStart();
        if (reset) {
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (reverse) {
            motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        while (opModeIsActive()) {
            if (set_power_mode_or_set_position_mode) {
                if (read_only)
                    motor0.setPower(0);
                else
                    motor0.setPower(max_power);
                motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry_M.addData("current",motor0.getCurrent(CurrentUnit.MILLIAMPS));
            } else {
                if (!read_only) {
                    motor0.setTargetPosition(encoder_position);
                    motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor0.setPower(max_power);
                }
                telemetry_M.addData("is busy", motor0.isBusy());
                telemetry_M.addData("encoder", motor0.getCurrentPosition());
            }
            telemetry_M.addData("velocity", motor0.getVelocity());
            telemetry_M.update();
        }
    }
}
