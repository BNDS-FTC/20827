package org.firstinspires.ftc.teamcode.PYZ;


import static org.firstinspires.ftc.teamcode.PYZ.PYZConfigurations.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Config
@TeleOp
public class XCYTeleOp extends LinearOpMode {

   private static final double max_turn_assist_power = 0.4;
   private NanoClock time;

   private double global_drive_power = 1;
   private double global_drive_turn_ratio = 1;
   private XCYSuperStructure upper;

   public static final double x_static_compensation = 0.06;
   public static final double y_static_compensation = 0.06;

   private PYZMecanumDrive drive;
   private Pose2d current_pos, lastEjectPos;
   private Vector2d cone_pos_vec = new Vector2d(50, -1200);
   double current_intake_length;
   private XCYBoolean to_last_eject;
   private List<LynxModule> allHubs;
   private boolean holding;
   private Sequence sequence;

   enum Sequence {
      EMPTY_MIDDLE, EMPTY_DOWN, HOLDING_AWAIT, HOLDING_UP
   }

   /**
    * <p>stick/trigger: 移动
    * </p><p>
    * a: 打开瞄准模式</p><p>
    * b: 夹取</p><p>
    * x: 释放/回收</p>
    * <p>y: 高位</p>
    * <p>dpad up: 中位</p>
    * <p>dpad down: 低位</p>
    * <p>dpad right/dpad left: 左右侧手</p><p>
    * bumper: 自动移动</p>
    * <p>home: 回正</p>
    * <p>touch pad: 换边</p>
    */

   @Override
   public void runOpMode() throws InterruptedException {
      time = NanoClock.system();
      allHubs = hardwareMap.getAll(LynxModule.class);
      sequence = Sequence.EMPTY_MIDDLE;
      upper = new XCYSuperStructure(
              this,
              new Runnable() {
                 @Override
                 public void run() {
                    logic_period();
                    drive_period();
                 }
              });
      drive = new PYZMecanumDrive(hardwareMap);
      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.getLocalizer().setPoseEstimate(get_pos_from_csv());
      current_intake_length = 420;
      holding = false;
      XCYBoolean arm_down = new XCYBoolean(() -> (gamepad2.cross || gamepad1.cross || (gamepad1.right_bumper && sequence == Sequence.EMPTY_MIDDLE)) && !gamepad1.share);
      XCYBoolean intake_action = new XCYBoolean(() -> (gamepad2.circle || gamepad1.circle || (gamepad1.right_bumper && sequence == Sequence.EMPTY_DOWN)) && !gamepad1.share);
      XCYBoolean upper_release = new XCYBoolean(() -> (gamepad2.square || gamepad1.square || (gamepad1.right_bumper && sequence == Sequence.HOLDING_UP)) && !gamepad1.share);
      XCYBoolean high_j = new XCYBoolean(() -> (gamepad2.triangle || gamepad1.triangle || (gamepad1.right_bumper && sequence == Sequence.HOLDING_AWAIT)) && !gamepad1.share);
      XCYBoolean toMiddle = new XCYBoolean(() -> gamepad1.options);
      XCYBoolean reset_imu = new XCYBoolean(() -> gamepad1.share && gamepad1.square);
      XCYBoolean switch_side = new XCYBoolean(() -> gamepad1.touchpad || gamepad2.touchpad || gamepad1.left_stick_button);

      to_last_eject = new XCYBoolean(() -> gamepad1.left_bumper && holding);
      XCYBoolean auto_intake = new XCYBoolean(() -> gamepad1.left_bumper && !holding);

      XCYBoolean low_j = new XCYBoolean(() -> !gamepad1.share && (gamepad2.dpad_down || gamepad1.dpad_down));
      XCYBoolean mid_j = new XCYBoolean(() -> !gamepad1.share && (gamepad2.dpad_up || gamepad1.dpad_up));
      XCYBoolean ground_j = new XCYBoolean(() -> !gamepad1.share &&(gamepad2.dpad_right || gamepad1.dpad_right));

//      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        upper.setAimReleaseCondition(dpad_down, dpad_up, upper_release, button_y);
//        upper.setWebcamConfig();
      for (LynxModule module : allHubs) {
         module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
      }

      Gamepad.LedEffect effect_holding = new Gamepad.LedEffect.Builder()
              .addStep(1, 0, 0, 65)
              .addStep(1, 1, 0, 65)
              .addStep(0, 1, 0, 65)
              .addStep(0, 1, 1, 65)
              .addStep(0, 0, 1, 65)
              .addStep(1, 0, 1, 65)
              .setRepeating(true)
              .build();
      Gamepad.LedEffect effect_idle = new Gamepad.LedEffect.Builder()
              .addStep(1, 0, 0, 100)
              .addStep(0.5, 0.5, 0, 100)
              .addStep(0, 1, 0, 100)
              .addStep(0, 0.5, 0.5, 100)
              .addStep(0, 0, 1, 100)
              .addStep(0.5, 0, 0.5, 100)
              .setRepeating(true)
              .build();
      gamepad1.runLedEffect(effect_idle);

      waitForStart();

      logic_period();

      int intakeAimPosIndex = 0;
      lastEjectPos = current_pos;

      while (opModeIsActive()) {

         if (auto_intake.toTrue() && !holding) {
            //自动夹取
            double target_angle = cone_pos_vec.minus(current_pos.vec()).angle();
            Vector2d intake_vec = new Vector2d(XCYSuperStructure.MIN_INTAKE_MOVE_DISTANCE, 0).rotated(target_angle);
            drive.initSimpleMove(new Pose2d(cone_pos_vec.minus(intake_vec), target_angle));
            XCYBoolean aim_ready = new XCYBoolean(() -> Math.abs(AngleUnit.normalizeRadians(current_pos.getHeading() - target_angle)) < Math.toRadians(13));
            logic_period();
            while (!aim_ready.get() && auto_intake.get()) {
               logic_period();
               drive_period3();
            }
            if (intakeAimPosIndex == 0) {
               upper.toAim();
            }
            long time = System.currentTimeMillis();
            while ((System.currentTimeMillis() - time < 200
                    || current_pos.minus(drive.getSimpleMovePosition()).vec().norm() > 40)
                    && auto_intake.get()) {
               logic_period();
               drive_period3();
            }
            time = System.currentTimeMillis();
            while (auto_intake.get() && System.currentTimeMillis() - time < 90) {
               logic_period();
               drive_period3();
            }
            while (auto_intake.get()) {
               logic_period();
               drive_period3();
            }
            global_drive_power = 0.7;
            global_drive_turn_ratio = 0.3;
            aim_ready.deactivate();
         } else {
            logic_period();
            drive_period();
         }

         if (reset_imu.toFalse()) {
            drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
         }

         if (arm_down.toTrue()) {
            if (gamepad1.right_bumper) gamepad1.rumble(0, 1, 200);
            global_drive_power = 0.7;
            global_drive_turn_ratio = 0.3;
            upper.toAim();
            intakeAimPosIndex = 0;
            upper.sleep_with_drive(300);
            holding = false;
            sequence = Sequence.EMPTY_DOWN;
         }

         if (toMiddle.toFalse()) {
            upper.toOrigional();
            upper.runtimeResetLifter();
         }

         if (intake_action.toTrue()) {
            //b
            if (holding) {
               upper.toGround();
               global_drive_power = 0.6;
               global_drive_turn_ratio = 0.3;
            } else {
               if (gamepad1.right_bumper) gamepad1.rumble(0, 1, 200);
               upper.grab();
               holding = true;
               sequence = Sequence.HOLDING_AWAIT;
               gamepad1.runLedEffect(effect_holding);
               upper.toOrigional();
               cone_pos_vec = getIntakePos();
               global_drive_power = 1;
               global_drive_turn_ratio = 1;
            }
         }

         if (!holding) {
            if (mid_j.toTrue()) {
               intakeAimPosIndex = Range.clip(intakeAimPosIndex + 1, 0, 4);
               upper.toAim(intakeAimPosIndex);
            } else if (low_j.toTrue()) {
               intakeAimPosIndex = Range.clip(intakeAimPosIndex - 1, 0, 4);
               upper.toAim(intakeAimPosIndex);
            }
         } else {
            if (switch_side.toTrue()) {
               upper.switchSide();
            }

            if (mid_j.toTrue() || low_j.toTrue() || high_j.toTrue() || ground_j.toTrue()) {
               if (gamepad1.right_bumper) gamepad1.rumble(0, 1, 200);
               if (mid_j.toTrue()) {
                  global_drive_power = 1;
                  global_drive_turn_ratio = 0.4;
                  upper.toMidJunction();
               } else if (low_j.toTrue()) {
                  global_drive_power = 1;
                  global_drive_turn_ratio = 0.4;
                  upper.toLowJunction();
               } else if (high_j.toTrue()) {
                  global_drive_power = 0.9;
                  global_drive_turn_ratio = 0.35;
                  upper.toHighJunction();
               } else if(ground_j.toTrue()){
                  global_drive_power = 0.9;
                  global_drive_turn_ratio = 0.35;
                  upper.toGroundJunction();
               }
               upper.sleep_with_drive(300);
               sequence = Sequence.HOLDING_UP;
               logic_period();
               while (upper_release.get() || high_j.get() || mid_j.get() || low_j.get() || ground_j.get()) {
                  logic_period();
                  drive_period();
               }
               while (!(upper_release.toTrue() || high_j.get() || mid_j.get() || low_j.get() || ground_j.get())) {
                  logic_period();
                  drive_period();
                  if (switch_side.toTrue()) upper.switchSide();
               }
               long startTime;
               if (upper_release.get())
                  do {
                     while (!(upper_release.toTrue() || high_j.get() || mid_j.get() || low_j.get() || ground_j.get())) {
                        logic_period();
                        drive_period();
                        if (switch_side.toTrue()) upper.switchSide();
                     }
                     startTime = System.currentTimeMillis();
                     if (upper_release.toTrue()) {
                        upper.armChange(-0.065);
                        while (upper_release.get()) {
                           logic_period();
                           drive_period();
                        }
                     }
                     if (System.currentTimeMillis() - startTime > 450)
                        upper.armChange(0.065);
                  } while (System.currentTimeMillis() - startTime > 450);
            }
         }

         if (upper_release.toFalse()) {
            //x
            if (holding) {
               if (gamepad1.right_bumper) gamepad1.rumble(0, 1, 200);
               lastEjectPos = current_pos;
               upper.openHand();
               upper.sleep_with_drive(250);
               upper.post_eject();
               upper.sleep_with_drive(300);
            } else {
               upper.setHand(0);
            }
            global_drive_turn_ratio = 1;
            global_drive_power = 1;
            holding = false;
            sequence = Sequence.EMPTY_MIDDLE;
            gamepad1.runLedEffect(effect_idle);
            upper.toOrigional();
            upper.runtimeResetLifter();
         }
      }


   }


   private double last_time_sec;
   private double period_time_sec;

   private void logic_period() {
      XCYBoolean.bulkRead();
      current_pos = drive.getPoseEstimate();
      period_time_sec = time.seconds() - last_time_sec;
      telemetry.addData("elapse time", period_time_sec * 1000);
      last_time_sec = time.seconds();
      telemetry.update();
      for (LynxModule module : allHubs) {
         module.clearBulkCache();
      }
   }

   private void drive_period() {
      if (to_last_eject.toTrue()) {
         drive.initSimpleMove(lastEjectPos);
      } else if (to_last_eject.get()) {
         drive_period2();
      } else {
         double x = -gamepad1.left_stick_y * 0.4 + -gamepad1.right_stick_y * 0.6;
         double y = -gamepad1.left_stick_x * 0.4 + -gamepad1.right_stick_x * 0.6;
         double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
         Vector2d fast_stick = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
         double corrected_rad = fast_stick.angle() - current_pos.getHeading();
         while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
         while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
         if (Math.abs(corrected_rad) < Math.PI / 5) {
            double div = clamp(
                    Math.toDegrees(corrected_rad) / 20, 1)
                    * max_turn_assist_power * fast_stick.norm();
            turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
         }
         Pose2d power = (new Pose2d(x, y, turn_val * global_drive_turn_ratio)).times(global_drive_power);

         drive.setGlobalPower(power, x_static_compensation, y_static_compensation);
         drive.update();
      }
   }

   private void drive_period2() {
      double x = -gamepad1.left_stick_y;
      double y = -gamepad1.left_stick_x;
      double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
      drive.setSimpleMovePosition(drive.getSimpleMovePosition().plus(new Pose2d(
              x * period_time_sec * 150,
              y * period_time_sec * 150,
              turn_val * period_time_sec * Math.toRadians(30)
      )));
      drive.update();
   }

   private void drive_period3() {
      double x = -gamepad1.left_stick_y;
      double y = -gamepad1.left_stick_x;
      drive.setSimpleMovePosition(new Pose2d(drive.getSimpleMovePosition().vec().plus(new Vector2d(
              x * period_time_sec * 150,
              y * period_time_sec * 150
      )),
              cone_pos_vec.minus(current_pos.vec()).angle()));
      drive.update();
   }

   private Vector2d getIntakePos() {
      return current_pos.vec().plus(new Vector2d(current_intake_length, 0).rotated(current_pos.getHeading()));
   }
}
