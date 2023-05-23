package org.firstinspires.ftc.teamcode.TestOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "read analog")
@Disabled
public class AnalogTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AnalogInput analog0= hardwareMap.get(AnalogInput.class,"analog0");
        AnalogInput analog1= hardwareMap.get(AnalogInput.class,"analog1");
        waitForStart();
        while (opModeIsActive()) {
            telemetry_M.addData("ana0",analog0.getVoltage());
            telemetry_M.addData("ana1",analog1.getVoltage());
            telemetry_M.update();
        }
    }
}
