package org.firstinspires.ftc.teamcode.TestOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "read color sensor")
@Config
public class ColorTest extends LinearOpMode {

   public static boolean side_red=false;
   @Override
   public void runOpMode() {
      Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
      ColorSensor color = hardwareMap.get(ColorSensor.class, "color");
      ColorSensor colorHigh = hardwareMap.get(ColorSensor.class, "color2");
      waitForStart();
      while (opModeIsActive()) {
         double r = color.red();
         double b = color.blue();
         double g = color.green();
         telemetry_M.addData("alpha2", colorHigh.alpha());
         telemetry_M.addData("alpha", color.alpha());
         telemetry_M.addData("r", r);
         telemetry_M.addData("b", b);
         telemetry_M.addData("g", g);
         telemetry_M.update();
      }
   }
}
