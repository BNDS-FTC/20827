package org.firstinspires.ftc.teamcode.PYZ;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class XCYSuperStructure {

   public static double HAND_GRAB = 0.15 ;//TODO
   public static double HAND_RELEASE = 0;

   public static double HAND_SPIN_FRONT = 0;
   public static double HAND_SPIN_BACK = 0.7;
   public static double HAND_SPIN_DIFF_SIDE = 0.;

   public static double ARM_INTAKE = 0.01; //TODO
   public static double ARM_FRONT = 0.32;
   public static double ARM_BACK = 0.7;
   public static double ARM_MIDDLE = 0.5;
   public static double AMR_EJECT_PUSH = 0.;

   public static int LIFT_MIN = 0;
   public static int LIFT_LOW = 250;
   public static int LIFT_MID = 650;
   public static int LIFT_HIGH = 1165;
   public static int LIFT_SWITCH_DIFF = 100;
   public static int LIFT_ADD_PER_CONE = 50;

   public static double GUIDE_DOWN = 1;
   public static double GUIDE_BACK = 0.42;

   public static final double MIN_INTAKE_MOVE_DISTANCE = 300;
   public static final int LIFT_TOLERANCE = 50;

   private final DcMotorEx liftMotorLeft, liftMotorRight;
   private final Servo handSpin, leftClaw, rightClaw, rightArm;
   private Runnable drive_period;
   private final LinearOpMode opMode;

   private int liftLocation;
   private boolean isHandFront;

   public XCYSuperStructure(LinearOpMode opMode,
                            Runnable drivePeriod) {
      drive_period = drivePeriod;
      this.opMode = opMode;
      HardwareMap hardwareMap = opMode.hardwareMap;

      liftMotorLeft = hardwareMap.get(DcMotorEx.class, "ll");
      liftMotorRight = hardwareMap.get(DcMotorEx.class, "lr");
      liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

      liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      handSpin = hardwareMap.get(Servo.class, "hand");
      liftLocation = 0;
      isHandFront = true;
      leftClaw = hardwareMap.get(Servo.class, "clawL");
      leftClaw.setDirection(Servo.Direction.REVERSE);
      rightClaw = hardwareMap.get(Servo.class, "clawR");
      rightClaw.setDirection(Servo.Direction.REVERSE);
      rightArm = hardwareMap.get(Servo.class, "armR");
      rightArm.setDirection(Servo.Direction.REVERSE);
   }

   public void toOrigional() {
      liftLocation = 0;
      setLifterPosition(LIFT_MIN, 1);
      setArm(ARM_MIDDLE);
      while (Math.abs(getLifterPos() - LIFT_MIN) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
   }

   public void runtimeResetLifter() {
      setLifterPower(-0.4);
      sleep_with_drive(200);
      setLifterPower(0);
      sleep_with_drive(100);
      setLifterPower(0);
      liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      setLifterPosition(0, 1);
   }

   public void toAim() {
      liftLocation = 0;
      isHandFront = true;
      setArm(ARM_INTAKE);
      openHand();
      setHandSpin(HAND_SPIN_FRONT);
      sleep_with_drive(50);
      setLifterPosition(LIFT_MIN, 1);
   }

   public void toAim(int index) {
      liftLocation = 0;
      isHandFront = true;
      setArm(ARM_INTAKE);
      setHand(HAND_RELEASE);
      setHandSpin(HAND_SPIN_FRONT);
      setLifterPosition(index * LIFT_ADD_PER_CONE, 1);
   }

   public void toGround() {
      liftLocation = 0;
      setLifterPosition(LIFT_MIN, 1);
      switchSide(true);
   }

   public void toGroundJunction(){
      liftLocation = 0;
      setLifterPosition(LIFT_MIN,1);
      switchSide(true);
      setArm(0.07);
   }

   public void toLowJunction() {
      liftLocation = 1;
      setLifterPosition(LIFT_LOW, 0.5);
      sleep_with_drive(400);
      switchSide(true);
   }

   public void toMidJunction() {
      liftLocation = 2;
      setLifterPosition(LIFT_MID, 1);
      switchSide(false);
   }

   public void toHighJunction() {
      liftLocation = 3;
      setLifterPosition(LIFT_HIGH, 1);
      switchSide(false);
   }

   public void switchSide() {
      if (liftLocation != 1) {
         if (isHandFront)
            setLifterPosition(liftMotorLeft.getCurrentPosition() - LIFT_SWITCH_DIFF, 1);
         else setLifterPosition(liftMotorLeft.getCurrentPosition() + LIFT_SWITCH_DIFF, 1);
      }
      switchSide(!isHandFront);
   }

   public void switchSide(boolean toFront) {
      if (toFront) {
         setArm(ARM_FRONT);
         sleep_with_drive(70);
         setHandSpin(HAND_SPIN_FRONT);
         isHandFront = true;
      } else {
         while (getLifterPos() < 0.6 * LIFT_LOW) drive_period.run();
         setArm(ARM_BACK);
         sleep_with_drive(70);
         setHandSpin(HAND_SPIN_BACK);
         isHandFront = false;
      }
   }

   public void setHandSpin(double val) {
      handSpin.setPosition(val);
   }

   public void grab() {
      if (opMode.isStopRequested()) return;
      closeHand();
      sleep_with_drive(350);
      setArm(ARM_MIDDLE);
      setHandSpin(HAND_SPIN_FRONT);
   }

   public void openHand() {
      if (opMode.isStopRequested()) return;
      setHand(HAND_RELEASE);
   }

   public void closeHand() {
      setHand(HAND_GRAB);
   }

   public void post_eject() {
      setLifterPower(0);
      setArm(ARM_MIDDLE);
   }

   public void setDrivePeriod(Runnable drivePeriod) {
      drive_period = drivePeriod;
   }

   public void setSideIsRed(boolean isRed) {

   }
   public void armChange(double val) {
      if (isHandFront)
         setArm(getArmPos() + val);
      else
         setArm(getArmPos() - val);
   }

   private void setLifterPosition(int pos, double power) {
      liftMotorLeft.setTargetPosition(pos);
      liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      liftMotorLeft.setPower(power);
      liftMotorRight.setTargetPosition(pos);
      liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      liftMotorRight.setPower(power);
   }

   private void setLifterPower(double power) {
      liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotorLeft.setPower(power);
      liftMotorRight.setPower(power);
      drive_period.run();
      liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotorLeft.setPower(power);
      liftMotorRight.setPower(power);
   }

   public double getLifterPos() {
      return (double) (liftMotorLeft.getCurrentPosition() + liftMotorRight.getCurrentPosition()) / 2;
   }

   public void setHand(double val) { //TODO
      leftClaw.setPosition(0.2 + val);
      rightClaw.setPosition(0.8 - val);
   }

   public void setArm(double val) {
      rightArm.setPosition(val);
   }

   public double getArmPos() {
      return rightArm.getPosition();
   }

   public void sleep_with_drive(double time_mm) {
      long start_time = System.currentTimeMillis();
      while (opMode.opModeIsActive() && System.currentTimeMillis() - start_time < time_mm) {
         drive_period.run();
      }
   }
}
