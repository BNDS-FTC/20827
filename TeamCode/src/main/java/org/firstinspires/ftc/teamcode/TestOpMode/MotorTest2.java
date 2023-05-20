package org.firstinspires.ftc.teamcode.TestOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "two motor test")
@Config
@Disabled
public class MotorTest2 extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
    public static int encoder_position=0;
    public static double max_power=0.6;
    public static boolean read_only=true;
    public static boolean reverse=true;
    public static boolean reset=true;
    public static String motor0_name ="intake0";
    public static String motor1_name="intake1";
    @Override
    public void runOpMode() {
        DcMotorEx motor0 = hardwareMap.get(DcMotorEx.class, motor0_name);
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, motor1_name);
        waitForStart();
        if (reset) {
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (reverse){
            motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        while (opModeIsActive()) {
            if (!read_only) {
                motor0.setTargetPosition(encoder_position);
                motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor0.setPower(max_power);
                motor1.setTargetPosition(encoder_position);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1.setPower(max_power);
            }
            telemetry_M.addData("encoder", motor0.getCurrentPosition());
            telemetry_M.addData("is busy", motor0.isBusy());
            telemetry_M.update();
        }
    }
}
