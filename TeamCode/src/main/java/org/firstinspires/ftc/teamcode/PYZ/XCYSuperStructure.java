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

   public static double HAND_GRAB = 0.25;//TODO
   public static double HAND_RELEASE = 0;
   public static double HAND_MIDDLE = 0.15;


   public static double HAND_SPIN_FRONT = 0.16;
   public static double HAND_SPIN_BACK = 0.84;

   public static double ARM_INTAKE = 0.01; //TODO
   public static double ARM_FRONT = 0.3;
   public static double ARM_BACK = 0.67;
   public static double ARM_HOLD = 0.65;
   public static double ARM_MIDDLE = 0.5;

   public static int LIFT_MIN = 0;
   public static int LIFT_HOLD =379;
   public static int LIFT_LOW = 170;
   public static int LIFT_MID = 780;
   public static int LIFT_HIGH = 1430;
   public static int LIFT_SWITCH_DIFF = 100;
   public static int LIFT_ADD_PER_CONE = 53;
//   public static int LIFT_INIT =
   public static final double MIN_INTAKE_MOVE_DISTANCE = 300;
   public static final int LIFT_TOLERANCE = 20;

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
      setArm(ARM_MIDDLE);
      setLifterPosition(LIFT_MIN, 1);
      while (Math.abs(getLifterPos() - LIFT_MIN) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
      openHand();
   }

   public void toHold() {
      liftLocation = 0;
      setLifterPosition(LIFT_HOLD, 1);
      while (Math.abs(getLifterPos() - LIFT_HOLD) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
      setArm(ARM_HOLD);
      sleep_with_drive(300);
      idleHand();
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
      openHand();
      setHandSpin(HAND_SPIN_FRONT);
      sleep_with_drive(30);
      setArm(ARM_INTAKE);
      setLifterPosition(LIFT_MIN, 1);
   }

   public void toAim(int index) {
      liftLocation = 0;
      isHandFront = true;
      setHand(HAND_RELEASE);
      setHandSpin(HAND_SPIN_FRONT);
      sleep_with_drive(30);
      setArm(ARM_INTAKE);
      setLifterPosition((index) * LIFT_ADD_PER_CONE, 1);
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
      sleep_with_drive(100);
      switchSide(true);
   }

   public void toMidJunction() {
      liftLocation = 2;
      setLifterPosition(LIFT_MID, 1);
      switchSide(false);
   }

   public void toHighJunction() {
      closeHand();
      liftLocation = 3;
      setLifterPosition(LIFT_HIGH, 1);
      sleep_with_drive(380);
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
         closeHand();
         sleep_with_drive(100);
         setArm(ARM_FRONT);
         sleep_with_drive(70);
         setHandSpin(HAND_SPIN_FRONT);
         isHandFront = true;
      } else {
         while (getLifterPos() < 0.6 * LIFT_LOW) drive_period.run();
         closeHand();
         sleep_with_drive(100);
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
      sleep_with_drive(300);
      setHandSpin(HAND_SPIN_FRONT);
      setArm(ARM_MIDDLE);
   }

   public void verticalGrab(){
      closeHand();
      sleep_with_drive(250);
      int target = (int)getLifterPos()+300;
      setLifterPosition(target,1);
      while (Math.abs(getLifterPos() - target) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
   }

   public void openHand() {
      setHand(HAND_RELEASE);
   }

   public void closeHand() {
      setHand(HAND_GRAB);
   }

   public void idleHand(){
      setHand(HAND_MIDDLE);
   }

   public void post_eject() {
      setLifterPower(0);
      setArm(ARM_HOLD);
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
