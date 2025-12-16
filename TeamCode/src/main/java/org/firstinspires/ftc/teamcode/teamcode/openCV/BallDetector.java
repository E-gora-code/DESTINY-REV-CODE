package org.firstinspires.ftc.teamcode.teamcode.openCV;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BallDetector extends OpenCvPipeline {

    public int ballCount = 0;

    public boolean hasThreeBalls = false;

    public Point ball1 = new Point(0, 0);


    public Point ball2 = new Point(0, 0);


    public Point ball3 = new Point(0, 0);


    public double color1 = 0;


    public double color2 = 0;


    public double color3 = 0;


    public double distance12 = 0;


    public double distance23 = 0;


    public double distance13 = 0;


    private Scalar lowerColor1 = new Scalar(0, 50, 50);
    private Scalar upperColor1 = new Scalar(30, 255, 255);
    private Scalar lowerColor2 = new Scalar(150, 50, 50);
    private Scalar upperColor2 = new Scalar(180, 255, 255);

    private double minBallArea = 100;

    private double maxBallArea = 5000;

    private double circularityThreshold = 0.7;

    private double maxBallDistance = 300;

    private Mat hsv = new Mat();
    private Mat colorMask1 = new Mat();
    private Mat colorMask2 = new Mat();
    private Mat morphologyKernel = new Mat();
    private Mat output = new Mat();

    private final Scalar ballColor = new Scalar(0, 255, 255);
    private final Scalar ball1Color = new Scalar(0, 255, 0);
    private final Scalar ball2Color = new Scalar(255, 0, 0);
    private final Scalar ball3Color = new Scalar(0, 0, 255);
    private final Scalar lineColor = new Scalar(255, 255, 0);



    @Override
    public Mat processFrame(Mat input) {

        resetResults();

        input.copyTo(output);

        List<Ball> balls = detectBalls(input);
        ballCount = balls.size();

        if (ballCount >= 3) {
            hasThreeBalls = true;

            Ball b1 = balls.get(0);
            Ball b2 = balls.get(1);
            Ball b3 = balls.get(2);

            // Сохраняем координаты и радиусы
            ball1 = b1.center;
            ball2 = b2.center;
            ball3 = b3.center;
            color1 = b1.colorType;
            color2 = b2.colorType;
            color3 = b3.colorType;

        }


        drawVisualization(output, balls);

        return output;
    }


    private List<Ball> detectBalls(Mat input) {
        List<Ball> balls = new ArrayList<>();
        List<BallContour> ballContours = new ArrayList<>();


        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);


        Core.inRange(hsv, lowerColor1, upperColor1, colorMask1);
        Core.inRange(hsv, lowerColor2, upperColor2, colorMask2);


        Mat colorMask = new Mat();
        Core.bitwise_or(colorMask1, colorMask2, colorMask);


        if (morphologyKernel.empty()) {
            morphologyKernel = Imgproc.getStructuringElement(
                    Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        }
        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, morphologyKernel);
        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_OPEN, morphologyKernel);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(colorMask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);


            if (area > minBallArea && area < maxBallArea) {

                Point center = new Point();
                float[] radius = new float[1];
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                Imgproc.minEnclosingCircle(contour2f, center, radius);


                double circleArea = Math.PI * radius[0] * radius[0];
                double circularity = area / circleArea;


                if (circularity > circularityThreshold) {

                    int colorType = determineBallColor(center);
                    ballContours.add(new BallContour(center, area, colorType));
                }
            }
        }


        ballContours.sort((b1, b2) -> Double.compare(b2.area, b1.area));


        for (int i = 0; i <= ballContours.size(); i++) {
            BallContour bc = ballContours.get(i);
            balls.add(new Ball(bc.center, bc.colorType));
        }


        contours.clear();
        hierarchy.release();
        colorMask.release();

        return balls;
    }


    private int determineBallColor(Point center) {

        Mat roi = new Mat(hsv, new Rect((int)center.x - 5, (int)center.y - 5, 10, 10));
        Scalar meanColor = Core.mean(roi);
        roi.release();


        double hue = meanColor.val[0];
        if (hue >= lowerColor1.val[0] && hue <= upperColor1.val[0]) {
            return 1;
        } else if (hue >= lowerColor2.val[0] && hue <= upperColor2.val[0]) {
            return 2;
        }
        return 0;
    }


    private void drawVisualization(Mat output, List<Ball> balls) {

        for (int i = 0; i < balls.size(); i++) {
            Ball ball = balls.get(i);
            Scalar color;
            String label;
            switch (ball.colorType) {
                case 1:
                    color = new Scalar(0, 255, 0);
                    label = "Color1";
                    break;
                case 2:
                    color = new Scalar(255, 0, 0);
                    label = "Color2";
                    break;
                default:
                    color = new Scalar(255, 255, 255);
                    label = "Unknown";
                    break;
            }

            Imgproc.circle(output, ball.center, 20, color, 3);
            Imgproc.circle(output, ball.center, 3, color, -1);


            Imgproc.putText(output, label,
                    new Point(ball.center.x + 25, ball.center.y),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
    }

    private void resetResults() {
        ballCount = 0;
        hasThreeBalls = false;
        ball1 = new Point(0, 0);
        ball2 = new Point(0, 0);
        ball3 = new Point(0, 0);
        color1 = 0;
        color2 = 0;
        color3 = 0;
        distance12 = 0;
        distance23 = 0;
        distance13 = 0;
    }

    private static class Ball {
        Point center;
        int colorType;

        Ball(Point center, int colorType) {
            this.center = center;
            this.colorType = colorType;
        }
    }
    private static class BallContour {
        Point center;
        double radius;
        double area;
        MatOfPoint contour;
        int colorType;

        BallContour(Point center, double area, int colorType) {
            this.center = center;
            this.radius = radius;
            this.area = area;
            this.contour = contour;
            this.colorType = colorType;
        }
    }
}