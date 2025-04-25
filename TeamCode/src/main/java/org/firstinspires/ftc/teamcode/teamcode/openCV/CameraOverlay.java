package org.firstinspires.ftc.teamcode.teamcode.openCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.sql.Timestamp;

public class CameraOverlay extends OpenCvPipeline {
    HSV hsv = new HSV();
    Mat mat = new Mat();
    Mat mask = new Mat();
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
        // Конвертируем в HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Mat output = input;

        long startTime = System.currentTimeMillis()/1000;

        if ((startTime%2)==0) {
            Imgproc.circle(output, new Point(300, 10), 5, new Scalar(255, 0, 0), -1);
        }


//        // Рисуем прямоугольник зоны анализа и центр масс (для визуализации)
//        Imgproc.rectangle(output, new Point(0, 100), new Point(320, 210), new Scalar(255, 0, 0), 1);
//        Imgproc.circle(output, centroid, 10, new Scalar(255, 0, 0), -1);

        // Освобождаем ресурсы (чтобы не было утечек памяти)
        mat.release();
        mask.release();
        submat.release();



        return output; // Возвращаем изображение с инвертированными областями
    }

    private Point Center(final Mat binaryImage) {
        final Moments moments = Imgproc.moments(binaryImage);
        if (moments.get_m00() == 0) {
            return new Point(binaryImage.width() / 2, binaryImage.height() / 2); // Центр камеры, если нет белых пикселей
        }
        final Point centroid = new Point();
        centroid.x = moments.get_m10() / moments.get_m00();
        centroid.y = moments.get_m01() / moments.get_m00();
        return centroid;
    }
}