package org.firstinspires.ftc.teamcode.PYZ;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;



public class XCYLocalizer implements Localizer {
    public static final double TICKS_PER_REV = 4096;
    public static final double WHEEL_RADIUS = 25.4; // mm

    public static final double FORWARD_OFFSET = -56; // mm; offset of the lateral wheel

    private final Encoder leftEncoder, rightEncoder, frontEncoder;
    private final BNO055IMU imu;

    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    private Pose2d poseVelocity = new Pose2d(0, 0, 0);

    private int last_right_pos, last_left_pos, last_front_pos;
    private final NanoClock time;
    private double last_time, last_rotation;
    private int rev_num = 0;

    public XCYLocalizer(HardwareMap hardwareMap, BNO055IMU imu) {
        this.imu = imu;
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rF"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "sb"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "nt"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
//        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        time = NanoClock.system();

        last_right_pos = rightEncoder.getCurrentPosition();
        last_left_pos = leftEncoder.getCurrentPosition();
        last_front_pos = frontEncoder.getCurrentPosition();
        last_time = time.seconds();
        last_rotation = imu.getAngularOrientation().firstAngle;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @NonNull
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    private double heading_rad_correct = 0;

    @Override
    public void update() {
        int current_right = rightEncoder.getCurrentPosition();
        int current_left = leftEncoder.getCurrentPosition();
        int current_front = frontEncoder.getCurrentPosition();
        double rotation = imu.getAngularOrientation().firstAngle - heading_rad_correct;
        double current_time = time.seconds();
        double corrected_rotation = rotation + Math.PI * 2 * rev_num;
        if (corrected_rotation - last_rotation > Math.PI) {
            rev_num--;
        } else if (corrected_rotation - last_rotation < -Math.PI) {
            rev_num++;
        }
        corrected_rotation = rotation + Math.PI * 2 * rev_num;

        int d_right = current_right - last_right_pos;
        int d_left = current_left - last_left_pos;
        int d_front = current_front - last_front_pos;
        double d_time = last_time - current_time;
        double d_rotation = corrected_rotation - last_rotation;

        last_right_pos = current_right;
        last_left_pos = current_left;
        last_front_pos = current_front;
        last_time = current_time;
        last_rotation = corrected_rotation;

        double d_x = encoderTicksToInches((d_left + d_right)) / 2;
        double d_y = encoderTicksToInches(d_front) - d_rotation * FORWARD_OFFSET;
        Vector2d d_pos = (new Vector2d(d_x, d_y)).rotated(corrected_rotation);

        poseEstimate = new Pose2d(poseEstimate.vec().plus(d_pos), rotation);
        poseVelocity = new Pose2d(d_pos.div(d_time), imu.getAngularVelocity().zRotationRate);
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d poseEstimate) {
        heading_rad_correct = imu.getAngularOrientation().firstAngle - poseEstimate.getHeading();
        this.poseEstimate = poseEstimate;
        last_rotation = poseEstimate.getHeading();
    }
}
