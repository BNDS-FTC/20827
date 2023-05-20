package org.firstinspires.ftc.teamcode.TestOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PYZ.PYZConfigurations.*;

@Autonomous(name = "reset all")
public class ResetOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx[] resetList = {hardwareMap.get(DcMotorEx.class, "rLift"),
                hardwareMap.get(DcMotorEx.class,"lLift"),
                hardwareMap.get(DcMotorEx.class, HardwareConstant.leftBackName),
                hardwareMap.get(DcMotorEx.class, HardwareConstant.rightBackName),
                hardwareMap.get(DcMotorEx.class, HardwareConstant.leftFrontName),
                hardwareMap.get(DcMotorEx.class, HardwareConstant.rightFrontName),
                hardwareMap.get(DcMotorEx.class, "slide")
        };

        waitForStart();
        if (isStopRequested()) return;

        for (DcMotorEx motor:resetList){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
