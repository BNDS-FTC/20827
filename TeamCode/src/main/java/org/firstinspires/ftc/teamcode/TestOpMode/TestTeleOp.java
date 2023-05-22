//package org.firstinspires.ftc.teamcode.TestOpMode;
//
//
//import static org.firstinspires.ftc.teamcode.PYZ.PYZConfigurations.*;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.PYZ.PYZMecanumDrive;
//import org.firstinspires.ftc.teamcode.PYZ.XCYBoolean;
//
//@TeleOp()
//public class TestTeleOp extends LinearOpMode {
//   private PYZMecanumDrive drive;
//
//   @Override
//   public void runOpMode() {
//      drive = new PYZMecanumDrive(hardwareMap);
//      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//      drive.getLocalizer().setPoseEstimate(get_pos_from_csv());
//      waitForStart();
//      XCYBoolean drive_auto_move = new XCYBoolean(() -> gamepad1.back);
//      Pose2d drive_pos = new Pose2d();
//      while (opModeIsActive()) {
//         XCYBoolean.bulkRead();
//         double x = -gamepad1.left_stick_y;
//         double y = -gamepad1.right_stick_x;
//         double turn_val = -gamepad1.right_trigger + gamepad1.left_trigger;
//         Pose2d power = (new Pose2d(x, y, turn_val)).times(0.4);
//
//         drive.setWeightedDrivePower(power);
//         drive.update();
//
//         if (gamepad1.start) {
//            drive.setPoseEstimate(new Pose2d());
//            drive.getLocalizer().setPoseEstimate(new Pose2d());
//            drive_pos = drive.getPoseEstimate();
//         }
//         if (drive_auto_move.toTrue()) {
//            drive.initSimpleMove(drive_pos);
//            while (drive_auto_move.get()) {
//               XCYBoolean.bulkRead();
//               drive.simpleMovePeriod();
//            }
//         }
//      }
//   }
//}
