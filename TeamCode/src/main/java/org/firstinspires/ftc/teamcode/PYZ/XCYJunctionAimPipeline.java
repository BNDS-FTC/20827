package org.firstinspires.ftc.teamcode.PYZ;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class XCYJunctionAimPipeline extends OpenCvPipeline {
    Mat process_mat = new Mat();
    Mat output_mat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    private boolean is_detected = false;
    private double junction_x_offset = 0;
    private double junction_distance = 0;
    public static int thresh = 90;
    public static int blur_pix = 5;
    public static int min_detect_area = 3000;
    public static double distance_const = 0.77;

    private boolean DEBUG = true;

    public static boolean cb_blur_img = false;
    public static boolean thresh_img = false;
    public static boolean contour = false;
    public static boolean data = false;

    public void setDEBUG(boolean b) {
        DEBUG = b;
    }

    @Override
    public Mat processFrame(Mat input) {
        contoursList.clear();
        Imgproc.cvtColor(input, process_mat, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(process_mat, process_mat, 2);

        Imgproc.blur(process_mat, process_mat, new Size(blur_pix, blur_pix));

        input.copyTo(output_mat);

        if (cb_blur_img && DEBUG)
            process_mat.copyTo(output_mat);

        Imgproc.threshold(process_mat, process_mat, thresh, 255, Imgproc.THRESH_BINARY_INV);

        if (thresh_img && DEBUG)
            process_mat.copyTo(output_mat);

        Imgproc.findContours(process_mat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//        Imgproc.er
//        Imgproc.fitLine();
        if (contour && DEBUG)
            Imgproc.drawContours(output_mat, contoursList, -1, new Scalar(0, 0, 255), 2, 8);

        contoursList.sort((o1, o2) -> (int) (Imgproc.contourArea(o2) - Imgproc.contourArea(o1)));

        if (contoursList.size() > 0 && Imgproc.contourArea(contoursList.get(0)) > min_detect_area) {
            RotatedRect rec = Imgproc.minAreaRect(new MatOfPoint2f(contoursList.get(0).toArray()));
            Point[] boxPoints = new Point[4];
            rec.points(boxPoints);
            setData(true,
                    process_mat.cols() * 0.5 - rec.center.x,
                    12.3d + 1d / Math.min(
                            getDistance(midpoint(boxPoints[0], boxPoints[1]), midpoint(boxPoints[2], boxPoints[3])),
                            getDistance(midpoint(boxPoints[3], boxPoints[0]), midpoint(boxPoints[1], boxPoints[2]))
                    ) * process_mat.cols() * distance_const
            );
        } else {
            setData(false, 0, getJunctionDistance());
        }

        if (data && DEBUG) {
            for (int c = 0; c < contoursList.size(); c++) {
                int area = (int) Imgproc.contourArea(contoursList.get(c));
                if (area < min_detect_area) {
                    continue;
                }

                RotatedRect rec = Imgproc.minAreaRect(new MatOfPoint2f(contoursList.get(c).toArray()));
                Point[] boxPoints = new Point[4];
                rec.points(boxPoints);

                Point pointA = midpoint(boxPoints[0], boxPoints[1]);
                Point pointB = midpoint(boxPoints[1], boxPoints[2]);
                Point pointC = midpoint(boxPoints[2], boxPoints[3]);
                Point pointD = midpoint(boxPoints[3], boxPoints[0]);

                for (int i = 0; i <= 3; i++) {
                    Imgproc.line(output_mat, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0));
                }
                Imgproc.putText(output_mat, "i=" + c + ",a=" + area + ",(" + getDistance(pointA, pointC) + "," + getDistance(pointD, pointB) + ")",
                        boxPoints[2], Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255));
            }
        }
        return output_mat;
    }

    private synchronized void setData(boolean detected, double offset, double distance) {
        is_detected = detected;
        junction_x_offset = offset;
        junction_distance = distance;
    }

    public synchronized boolean isDetected() {
        return is_detected;
    }

    public synchronized double getJunctionOffset() {
        return junction_x_offset;
    }

    public synchronized double getJunctionDistance() {
        return junction_distance;
    }

    private Point midpoint(Point ptA, Point ptB) {
        return new Point((ptA.x + ptB.x) * 0.5, (ptA.y + ptB.y) * 0.5);
    }

    private int getDistance(Point pointA, Point pointB) {
        return (int) (Math.hypot(pointA.x - pointB.x, pointA.y - pointB.y));
    }
}
