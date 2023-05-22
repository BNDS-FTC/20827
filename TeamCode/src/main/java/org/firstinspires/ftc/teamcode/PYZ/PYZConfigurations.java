package org.firstinspires.ftc.teamcode.PYZ;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;

@Config
public class PYZConfigurations {
    public static final class HardwareConstant {
        public static final String imuName = "imu";
        public static final String leftFrontName = "lF";
        public static final DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
        public static final String leftBackName = "lB";
        public static final DcMotorSimple.Direction leftBackDirection = DcMotorSimple.Direction.REVERSE;
        public static final String rightBackName = "rB";
        public static final DcMotorSimple.Direction rightBackDirection = DcMotorSimple.Direction.FORWARD;
        public static final String rightFrontName = "rF";
        public static final DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;

        public static final String xEncoderName = "rF";
        public static final Encoder.Direction xEncoderDirection = Encoder.Direction.FORWARD;
        public static final String yEncoderName = "rB";
        public static final Encoder.Direction yEncoderDirection = Encoder.Direction.REVERSE;

        public static final String lLiftEncoderName = "lLift";

        public static final DcMotorSimple.Direction lLiftDirection = DcMotorSimple.Direction.FORWARD;

        public static final String rLiftEncoderName = "rLift";

        public static final DcMotorSimple.Direction rLiftDirection = DcMotorSimple.Direction.REVERSE;

    }

    public static final double TICKS_PER_REV = 560;

    public static final double WHEEL_RADIUS = 50.8; // mm
    public static final double TRACK_WIDTH = 335; // mm
    public static final double WHEEL_BASE = 270; // mm

    public static final double MAX_VEL = 1600;                        // 底盘速度 todo
    public static final double MAX_ACCEL = 1400;
    public static final double MAX_ANG_VEL = Math.toRadians(100);
    public static double MAX_ANG_ACCEL = Math.toRadians(100);

    public static double kA = 0.00008; //todo
    public static double kStatic = 0.03;//0.05
    public static double kV = 0.0004; //0.00035 //0.00063

    public static double encoderTicksToMM(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static final String path = AppUtil.ROOT_FOLDER + "/RoadRunner/position.csv";

    public static void save_pos_in_csv(Pose2d pos) {
        try {
            BufferedWriter out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(path), StandardCharsets.UTF_8));
            out.write(pos.getX() + "," + pos.getY() + "," + pos.getHeading() + ",");
            out.flush();
            out.close();
        } catch (Exception ignored) {
        }
    }

    @NonNull
    public static Pose2d get_pos_from_csv() {
        try {
            BufferedReader in = new BufferedReader(new InputStreamReader(new FileInputStream(path), StandardCharsets.UTF_8));
            String[] line = in.readLine().split(",");
            in.close();
            return new Pose2d(Double.parseDouble(line[0]), Double.parseDouble(line[1]), Double.parseDouble(line[2]));
        } catch (Exception ignored) {
            return new Pose2d(-1, -1, -1);
        }
    }

    public static double clamp(double val, double limit) {
        return Range.clip(val, -limit, limit);
    }

    public  static class TimeoutException extends InterruptedException{
        public TimeoutException(){
            super();
        }
    }

    public static class StructureJamException extends InterruptedException{
        public StructureJamException(){
            super();
        }
    }

    public static class CollisionException extends InterruptedException{
        public CollisionException(){
            super();
        }
    }

    public static class ConeException extends InterruptedException{
        public ConeException(){
            super();
        }
    }

    public static class GlobalTimeoutException extends InterruptedException{
        public GlobalTimeoutException(){
            super();
        }
    }
}
