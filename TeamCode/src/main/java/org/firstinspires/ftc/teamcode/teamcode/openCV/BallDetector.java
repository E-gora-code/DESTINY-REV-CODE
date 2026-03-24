package org.firstinspires.ftc.teamcode.teamcode.openCV;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class BallDetector extends OpenCvPipeline {
    public List<Circle> circles = new ArrayList<>();
    public int ballCount = 0;

    private Mat gray;
    private Mat blurred;
    private Mat mat;
    private Mat mask;
    private Mat hierarchy;

    private Scalar color1 = new Scalar(0, 255, 0);
    private Scalar color2 = new Scalar(0, 165, 255);
    private Scalar lowerColor1 = new Scalar(70, 80, 30);
    private Scalar upperColor1 = new Scalar(90, 255, 255);
    private Scalar lowerColor2 = new Scalar(100, 40, 60);
    private Scalar upperColor2 = new Scalar(255, 140, 255);

    @Override
    public void init(Mat firstFrame) {
        releaseMats();
        createMats(firstFrame.size());
    }

    private void createMats(Size size) {
        gray = new Mat(size, CvType.CV_8UC1);
        blurred = new Mat(size, CvType.CV_8UC1);
        mat = new Mat(size, CvType.CV_8UC3);
        mask = new Mat(size, CvType.CV_8UC1);
        hierarchy = new Mat();
    }

    private void releaseMats() {
        if (gray != null) gray.release();
        if (blurred != null) blurred.release();
        if (mat != null) mat.release();
        if (mask != null) mask.release();
        if (hierarchy != null) hierarchy.release();
        gray = null;
        blurred = null;
        mat = null;
        mask = null;
        hierarchy = null;
    }

    @Override
    public Mat processFrame(Mat input) {
        circles.clear();
        ballCount = 0;

        if (mat == null || input.width() != mat.width() || input.height() != mat.height()) {
            releaseMats();
            createMats(input.size());
        }

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat mask1 = new Mat(input.size(), CvType.CV_8UC1);
        Mat mask2 = new Mat(input.size(), CvType.CV_8UC1);

        Core.inRange(mat, lowerColor1, upperColor1, mask1);
        Core.inRange(mat, lowerColor2, upperColor2, mask2);
        Core.bitwise_or(mask1, mask2, mask);
        mask1.release();
        mask2.release();

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 1000) {
                Rect rect = Imgproc.boundingRect(contour);
                int padding = 10;
                int x1 = Math.max(0, rect.x - padding);
                int y1 = Math.max(0, rect.y - padding);
                int x2 = Math.min(input.width(), rect.x + rect.width + padding);
                int y2 = Math.min(input.height(), rect.y + rect.height + padding);

                if (x2 - x1 > 20 && y2 - y1 > 20) {
                    Rect roiRect = new Rect(x1, y1, x2 - x1, y2 - y1);
                    Mat roiGray = mask.submat(roiRect);

                    Mat circlesMat = new Mat();
                    Imgproc.HoughCircles(roiGray, circlesMat, Imgproc.HOUGH_GRADIENT,
                            1, 18, 100, 8, 20, 35);

                    if (circlesMat.cols() > 0) {
                        for (int i = 0; i < circlesMat.cols(); i++) {
                            double[] data = circlesMat.get(0, i);
                            if (data != null && data.length >= 3) {
                                Point center = new Point(data[0] + x1, data[1] + y1);
                                double radius = data[2];
                                int range = getRange(mat, center, 5);
                                circles.add(new Circle(center, radius, range));

                                Scalar circleColor = range == 1 ? color1 : range == 2 ? color2 : new Scalar(255, 0, 0);
                                String text = range == 1 ? "R1" : range == 2 ? "R2" : "?";

                                Imgproc.circle(input, center, (int)radius, circleColor, 3);
                                Imgproc.circle(input, center, 2, new Scalar(255, 255, 255), 3);
                                Imgproc.putText(input, text, new Point(center.x + radius + 5, center.y),
                                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, circleColor, 2);
                            }
                        }
                    }
                    circlesMat.release();
                    roiGray.release();
                }
            }
            contour.release();
        }

        ballCount = circles.size();
        return input ;
    }

    private int getRange(Mat hsvImage, Point center, int radius) {
        int size = Math.max(3, radius / 4);
        int x = (int)Math.max(0, center.x - size);
        int y = (int)Math.max(0, center.y - size);
        int w = Math.min(2 * size, hsvImage.width() - x);
        int h = Math.min(2 * size, hsvImage.height() - y);

        if (w <= 0 || h <= 0) return 0;

        Mat region = hsvImage.submat(new Rect(x, y, w, h));
        Mat m1 = new Mat();
        Mat m2 = new Mat();

        Core.inRange(region, lowerColor1, upperColor1, m1);
        Core.inRange(region, lowerColor2, upperColor2, m2);

        int c1 = Core.countNonZero(m1);
        int c2 = Core.countNonZero(m2);

        m1.release();
        m2.release();
        region.release();

        if (c1 > c2 && c1 > w * h * 0.3) return 1;
        if (c2 > c1 && c2 > w * h * 0.3) return 2;
        return 0;
    }

    public static class Circle {
        public Point center;
        public double radius;
        public int range;

        public Circle(Point center, double radius, int range) {
            this.center = center;
            this.radius = radius;
            this.range = range;
        }
    }
}