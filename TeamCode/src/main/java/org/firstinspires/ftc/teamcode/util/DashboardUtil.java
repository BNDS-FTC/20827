package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0*25.4; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in
    private static final double inchesToMM = 25.4;

    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX()/inchesToMM;
            yPoints[i] = pose.getY()/inchesToMM;
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX()/inchesToMM;
            yPoints[i] = pose.getY()/inchesToMM;
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX()/inchesToMM, pose.getY()/inchesToMM, ROBOT_RADIUS);
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX()/inchesToMM + v.getX() / 2, y1 = pose.getY()/inchesToMM + v.getY() / 2;
        double x2 = pose.getX()/inchesToMM + v.getX(), y2 = pose.getY()/inchesToMM + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}
