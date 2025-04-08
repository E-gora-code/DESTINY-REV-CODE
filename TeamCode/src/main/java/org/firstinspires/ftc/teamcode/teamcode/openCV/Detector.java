package org.firstinspires.ftc.teamcode.teamcode.openCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

public class Detector extends OpenCvPipeline {
    HSV hsv = new HSV();
    Mat mat = new Mat();
    Mat submat = new Mat();
    public double fillValue = 0;
    public int center = 0;
    Point centroid;
    double hsvlh = hsv.lh;
    double hsvls = hsv.ls;
    double hsvlv = hsv.lv;
    double hsvhh = hsv.hh;
    double hsvhs = hsv.hs;
    double hsvhv = hsv.hv;
    // Метод для установки значений HSV
    public void setHSV(double lh, double ls, double lv, double hh, double hs, double hv) {
        hsvlh = lh;
        hsvls = ls;
        hsvlv = lv;
        hsvhh = hh;
        hsvhs = hs;
        hsvhv = hv;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // Перевод из RGB в HSV
        Scalar lhsv = new Scalar(hsvlh, hsvls, hsvlv); // Создает переменную для нижних значений HSV
        Scalar hhsv = new Scalar(hsvhh, hsvhs, hsvhv); // Создает переменную для верхних значений HSV
        Core.inRange(mat, lhsv, hhsv, mat); // Накладывание маски
        centroid = Center(mat);
        center = (int) centroid.x;
        submat = mat.submat(new Rect(new Point(0, 100), new Point(320, 210)));
        fillValue = Core.sumElems(submat).val[0] / new Rect(new Point(0, 100), new Point(320, 210)).area() / 255;
        Imgproc.rectangle(mat, new Point(0, 100), new Point(320, 210), new Scalar(255, 0, 0));
        Imgproc.circle(mat, centroid, 10, new Scalar(255, 0, 0), -1); // Рисование центра скопления точек
        return mat;
    }

    private Point Center(final Mat map) {
        final Moments moments = Imgproc.moments(map);
        final Point centroid = new Point();
        centroid.x = moments.get_m10() / moments.get_m00();
        return centroid;
    }
}
