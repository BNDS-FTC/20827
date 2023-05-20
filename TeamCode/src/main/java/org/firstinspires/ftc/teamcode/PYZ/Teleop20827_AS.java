package org.firstinspires.ftc.teamcode.PYZ;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PYZ.XCYBoolean;
import org.firstinspires.ftc.teamcode.PYZ.PYZMecanumDrive;

@TeleOp(name="Teleop20827_AS", group="Linear Opmode")
public class Teleop20827_AS extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotorEx liftMotorLeft = null;
    private DcMotorEx liftMotorRight = null;

    private Servo rightClaw = null;
    private Servo leftClaw = null;
    private Servo rightArm = null;
    private Servo leftArm = null;
    private Servo hand = null;
    private Servo back = null;

    private double i=0;

    Double width = 16.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    Double conversion = cpi * bias;
    Boolean exit = false;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


    private XCYBoolean dpad_down;
    private XCYBoolean dpad_up;
    private XCYBoolean right_bumper;
    int intake_pos_index = 0;
    //arm分别抓五个桶时的高度（列表）
    public static int[] intakeArmPositions = new int[]{100, 150, 200, 250, 300};

    PYZMecanumDrive drive;


    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // telemetry.addData(DcMot)
        drive = new PYZMecanumDrive(hardwareMap);
//        rightFrontDrive  = hardwareMap.get(DcMotor.class, "rf");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        liftMotorLeft = hardwareMap.get(DcMotorEx.class, "ll");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "lr");
        back = hardwareMap.get(Servo.class,"back");
        leftClaw = hardwareMap.get(Servo.class, "clawL");
        rightClaw = hardwareMap.get(Servo.class, "clawR");
        leftArm = hardwareMap.get(Servo.class, "armL");
        rightArm = hardwareMap.get(Servo.class, "armR");
        hand = hardwareMap.get(Servo.class, "hand");

//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        liftMotorRight.setDirection(DcMotor.Direction.REVERSE);


//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        rightArm.setDirection(Servo.Direction.REVERSE);

        double ly, rx, tr, denominator, frontLeftPower, backLeftPower, speed, frontRightPower, backRightPower, speedRatio,k;
        leftArm.setPosition(0.2);
        rightArm.setPosition(0.8);
        hand.setPosition(0);
        back.setPosition(0.34);
        claw_open();
        dpad_down = new XCYBoolean(() -> gamepad2.dpad_down);
        dpad_up = new XCYBoolean(() -> gamepad2.dpad_up);
        right_bumper = new XCYBoolean(()-> gamepad1.right_bumper);
        waitForStart();

        while (opModeIsActive())
        {
            XCYBoolean.bulkRead();

            speed=0;
            exit=false;
            int target=20;
            //滑轨目标位置

            if (!(gamepad2.right_bumper)) {
                speedRatio = 0.4;
                k=1.5;
            }
            else {
                speedRatio  = 0.75;
                k=1.0;
            }

//            ly = (-gamepad2.left_stick_y) * 0.90;
//            rx = (gamepad2.right_stick_x) * 0.90;
//            tr = (-gamepad2.left_trigger + gamepad2.right_trigger) * 0.75; //灵敏度
//            denominator = Math.max(Math.abs(ly) + Math.abs(rx) + Math.abs(tr), 1);
//
//            frontLeftPower = (ly + rx*k + tr) / denominator;
//            backLeftPower = (ly - rx*k + tr) / denominator;
//            frontRightPower = (ly + rx*k - tr) / denominator;
//            backRightPower = (ly - rx*k - tr) / denominator;
//
//            leftFrontDrive.setPower(frontLeftPower * speedRatio);
//            leftBackDrive.setPower(backLeftPower* speedRatio);
//            rightFrontDrive.setPower(frontRightPower * speedRatio);
//            rightBackDrive.setPower(backRightPower * speedRatio);

            double x = (-gamepad2.left_stick_y) * 0.5 + (-gamepad2.right_stick_y) * 0.3;  // fast stick + slow stick = 1.0 = speed
            double y = (-gamepad2.left_stick_x) * 0.5 + (-gamepad2.right_stick_x) * 0.3;
            double turn_val = (this.gamepad2.left_trigger - this.gamepad2.right_trigger) * 0.60;
            Vector2d fast_stick = new Vector2d(gamepad2.right_stick_y, gamepad2.right_stick_x);
            double corrected_rad = fast_stick.angle() - drive.getPoseEstimate().getHeading();
            while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
            while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
            if (Math.abs(corrected_rad) < Math.PI / 6) {
                double div = clamp(Math.toDegrees(corrected_rad) / 20, 1) * 0.3 * fast_stick.norm();
                turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
            }
            Pose2d power = (new Pose2d(-x, -y, -turn_val));
            drive.setGlobalPower(power,0.06,0.06);

            //滑轨
            if(gamepad1.dpad_up){
                target = 1250;
                speed = 1.0;     // original 0.9
                liftMotorLeft.setTargetPosition(target);
                liftMotorRight.setTargetPosition(target);

                liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setPower(speed);
                liftMotorRight.setPower(speed);
            }


            else if(gamepad1.dpad_left){
                target = 700;
                speed = 1.0;


                liftMotorLeft.setTargetPosition(target);
                liftMotorRight.setTargetPosition(target);

                liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setPower(speed);
                liftMotorRight.setPower(speed);
            }
            //

            else if (gamepad1.dpad_down){
                target = 0;
                speed = 1.0;    // original -0.9


                liftMotorRight.setTargetPosition(target);
                liftMotorLeft.setTargetPosition(target);

                liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setPower(speed);
                liftMotorRight.setPower(speed);
            }
            else if(gamepad1.dpad_right){
                target=150;
                speed=1.0;
                liftMotorLeft.setTargetPosition(target);
                liftMotorRight.setTargetPosition(target);

                liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setPower(speed);
                liftMotorRight.setPower(speed);
            }
//            else if(gamepad1.right_bumper){
//                target = 300;
//                speed = 1.0;
//                liftMotorLeft.setTargetPosition(target);
//                liftMotorRight.setTargetPosition(target);
//
//                liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotorLeft.setPower(speed);
//                liftMotorRight.setPower(speed);
//            }
            if(right_bumper.toTrue()){
                if(i==0){
                    back.setPosition(0.37);
                    i=1;
                }
                else{
                    back.setPosition(1);
                    i=0;
                }
            }

            // 爪子
            if(gamepad1.left_bumper){
                claw_open();
            }
            else{
                claw_close();
            }


            if(gamepad1.x){
                leftArm.setPosition(0.7);
                rightArm.setPosition(0.3);
                hand.setPosition(0.64);

            }
            else if(gamepad1.y){
                leftArm.setPosition(1);
                rightArm.setPosition(0);
                sleep(100);
                hand.setPosition(0.64);
                if(gamepad1.left_bumper){
                    claw_open();
                }
                else{
                    claw_close();
                }
            }
            else if(gamepad1.b){
                leftArm.setPosition(0.2);
                rightArm.setPosition(0.8);
                hand.setPosition(0);
            }

            else{
                if(gamepad1.left_bumper){
                    claw_open();
                }
                else{
                    claw_close();
                }
                // leftArm.setPosition(0.107);
                // rightArm.setPosition(0.107+0.082);
                // hand.setPosition(0.02);


            }

            //抓杯五档
            if (dpad_up.toTrue()) {
                if (intake_pos_index < 4){
                    intake_pos_index++;
                }
                target = intakeArmPositions[intake_pos_index];    // intakeArmPositions是一个存有五个位置高度的数组
                speed=1.0;
                liftMotorLeft.setTargetPosition(target);
                liftMotorRight.setTargetPosition(target);

                liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setPower(speed);
                liftMotorRight.setPower(speed);
            }
            //设置爪子到指定位置的动作
            else if (dpad_down.toTrue()) {
                if (intake_pos_index > 0){
                    intake_pos_index--;
                }

                target = intakeArmPositions[intake_pos_index];
                speed=1.0;
                liftMotorLeft.setTargetPosition(target);
                liftMotorRight.setTargetPosition(target);

                liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotorLeft.setPower(speed);
                liftMotorRight.setPower(speed);
            }
            //设置爪子到指定位置的动
            drive.update();
//            telemetry.addData("rightBackDrive", rightBackDrive.getCurrentPosition());
            telemetry.addData("liftLeft_Val", liftMotorLeft.getCurrentPosition());
            telemetry.addData("liftRight_Val", liftMotorRight.getCurrentPosition());
            telemetry.update();
        }
    }
    public void claw_open(){
        leftClaw.setPosition(0.2);
        rightClaw.setPosition(0.8);
    }
    public void claw_close(){
        leftClaw.setPosition(0.37);
        rightClaw.setPosition(0.63);   //0.78
    }
    public static double clamp(double val, double limit) {
        return Range.clip(val, -limit, limit);
    }

}

