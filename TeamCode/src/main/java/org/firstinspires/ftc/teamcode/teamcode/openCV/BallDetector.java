package org.firstinspires.ftc.teamcode.teamcode.openCV;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BallDetector extends OpenCvPipeline {

    public int ballCount = 0;
    public boolean hasThreeBalls = false;
    Point ball1, ball2, ball3;
    public double color1 = 0;
    public double color2 = 0;
    public double color3 = 0;
    public double distance12 = 0;
    public double distance23 = 0;
    public double distance13 = 0;

    private Scalar lowerColor1 = new Scalar(70, 80, 30);
    private Scalar upperColor1 = new Scalar(90, 255, 255);
    private Scalar lowerColor2 = new Scalar(100, 40, 60);
    private Scalar upperColor2 = new Scalar(255, 140, 255);

    private double minBallArea = 800;
    private double maxBallArea = 8000;

    private Mat hsv = new Mat();
    private Mat colorMask1 = new Mat();
    private Mat colorMask2 = new Mat();
    private Mat combinedMask = new Mat(); // Добавлено: объединенная маска для всего
    private Mat morphologyKernel = new Mat();
    private Mat output = new Mat();

    private final Scalar ballColor = new Scalar(0, 255, 255);
    private final Scalar ball1Color = new Scalar(0, 255, 0);
    private final Scalar ball2Color = new Scalar(255, 0, 0);
    private final Scalar ball3Color = new Scalar(0, 0, 255);
    private final Scalar lineColor = new Scalar(255, 255, 0);

    // Конструктор
    public BallDetector() {
        resetResults();
    }

    @Override
    public Mat processFrame(Mat input) {
        try {
            resetResults();


            if (input.empty()) {
                return input;
            }

            input.copyTo(output);

            List<Ball> balls = detectBalls(input);
            ballCount = balls.size();


            try {
                if (ballCount >= 3) {
                    hasThreeBalls = true;

                    Ball b1 = balls.get(0);
                    Ball b2 = balls.get(1);
                    Ball b3 = balls.get(2);

                    ball1 = b1.center;
                    ball2 = b2.center;
                    ball3 = b3.center;
                    color1 = b1.colorType;
                    color2 = b2.colorType;
                    color3 = b3.colorType;

                } else if (ballCount >= 2) {
                    hasThreeBalls = false;

                    Ball b1 = balls.get(0);
                    Ball b2 = balls.get(1);

                    ball1 = b1.center;
                    ball2 = b2.center;
                    color1 = b1.colorType;
                    color2 = b2.colorType;

                } else if (ballCount >= 1) {
                    hasThreeBalls = false;

                    Ball b1 = balls.get(0);

                    ball1 = b1.center;
                    color1 = b1.colorType;
                }
            } catch (IndexOutOfBoundsException e) {

                ballCount = Math.min(ballCount, balls.size());
            }


            drawVisualization(output, balls);

        } catch (Exception e) {


            return input;
        }

        return output;
    }




    private void applyColorMasking(Mat frame, Scalar color) {
        try {
            if (frame.empty() || combinedMask.empty()) {
                return;
            }


            Mat colored = new Mat(frame.size(), frame.type(), color);

            // Применяем маску: закрашиваем только области внутри маски
            colored.copyTo(frame, combinedMask);

            colored.release();

        } catch (Exception e) {

        }
    }

    private List<Ball> detectBalls(Mat input) {
        List<Ball> balls = new ArrayList<>();
        List<BallContour> ballContours = new ArrayList<>();

        try {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsv, lowerColor1, upperColor1, colorMask1);
            Core.bitwise_or(colorMask1, colorMask1, combinedMask);

            Mat colorMask = new Mat();
            combinedMask.copyTo(colorMask);

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
                try {
                    double area = Imgproc.contourArea(contour);

                    if (area > minBallArea && area < maxBallArea) {
                        Point center = new Point();
                        float[] radius = new float[1];
                        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                        Imgproc.minEnclosingCircle(contour2f, center, radius);

                        // Проверяем, что радиус больше 0
                        if (radius[0] <= 0) {
                            continue;
                        }

                        int colorType = determineBallColor(center);
                        ballContours.add(new BallContour(center, area, colorType));
                    }
                } catch (Exception e) {

                    continue;
                }
            }

            ballContours.sort((b1, b2) -> Double.compare(b2.area, b1.area));

            for (int i = 0; i < ballContours.size(); i++) {
                try {
                    BallContour bc = ballContours.get(i);
                    balls.add(new Ball(bc.center, bc.colorType));
                } catch (Exception e) {

                }
            }

            
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);


            Core.inRange(hsv, lowerColor2, upperColor2, colorMask2);
            Core.bitwise_or(colorMask2, colorMask2, combinedMask);

            colorMask = new Mat();
            combinedMask.copyTo(colorMask); // Используем копию для обработки

            if (morphologyKernel.empty()) {
                morphologyKernel = Imgproc.getStructuringElement(
                        Imgproc.MORPH_ELLIPSE, new Size(5, 5));
            }
            Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, morphologyKernel);
            Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_OPEN, morphologyKernel);

            contours = new ArrayList<>();
            hierarchy = new Mat();
            Imgproc.findContours(colorMask, contours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                try {
                    double area = Imgproc.contourArea(contour);

                    if (area > minBallArea && area < maxBallArea) {
                        Point center = new Point();
                        float[] radius = new float[1];
                        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                        Imgproc.minEnclosingCircle(contour2f, center, radius);

                        // Проверяем, что радиус больше 0
                        if (radius[0] <= 0) {
                            continue;
                        }

                        int colorType = determineBallColor(center);
                        ballContours.add(new BallContour(center, area, colorType));
                    }
                } catch (Exception e) {

                    continue;
                }
            }

            ballContours.sort((b1, b2) -> Double.compare(b2.area, b1.area));

            for (int i = 0; i < ballContours.size(); i++) {
                try {
                    BallContour bc = ballContours.get(i);
                    balls.add(new Ball(bc.center, bc.colorType));
                } catch (Exception e) {

                }
            }

            // Освобождаем ресурсы
            try {
                contours.clear();
                if (!hierarchy.empty()) hierarchy.release();
                if (!colorMask.empty()) colorMask.release();
            } catch (Exception e) {

            }


        } catch (Exception e) {

            return new ArrayList<>();
        }

        return balls;
    }

    private int determineBallColor(Point center) {
        try {

            if (hsv.empty() || center == null) {
                return 0;
            }

            int x = (int) center.x;
            int y = (int) center.y;
            int roiSize = 5;

            // Проверяем, что ROI не выходит за границы изображения
            if (x - roiSize < 0 || x + roiSize >= hsv.cols() ||
                    y - roiSize < 0 || y + roiSize >= hsv.rows()) {
                return 0;
            }

            Mat roi = new Mat(hsv, new Rect(x - roiSize, y - roiSize, roiSize * 2, roiSize * 2));
            Scalar meanColor = Core.mean(roi);
            roi.release();

            double hue = meanColor.val[0];
            if (hue >= lowerColor1.val[0] && hue <= upperColor1.val[0]) {
                return 1;
            } else if (hue >= lowerColor2.val[0] && hue <= upperColor2.val[0]) {
                return 2;
            }
            return 0;

        } catch (Exception e) {

            return 0;
        }
    }

    private void drawVisualization(Mat output, List<Ball> balls) {
        try {
            for (int i = 0; i < balls.size(); i++) {
                try {
                    Ball ball = balls.get(i);
                    if (ball == null || ball.center == null) {
                        continue;
                    }

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
                } catch (Exception e) {


                }
            }
        } catch (Exception e) {

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

        // Очищаем маски
        if (!combinedMask.empty()) {
            combinedMask.release();
        }
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
        double area;
        int colorType;

        BallContour(Point center, double area, int colorType) {
            this.center = center;
            this.area = area;
            this.colorType = colorType;
        }
    }

    // Метод для изменения типа закрашивания (опционально)
    public void setMaskingMode(boolean inverseMode) {
        // Можно добавить переключение между разными режимами закрашивания
    }
}