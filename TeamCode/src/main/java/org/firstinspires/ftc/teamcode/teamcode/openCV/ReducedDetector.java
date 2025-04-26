package org.firstinspires.ftc.teamcode.teamcode.openCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

public class ReducedDetector extends OpenCvPipeline {
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

        // Создаём маску (чёрно-белую, где белое = попадает в диапазон)
        Scalar lhsv = new Scalar(hsvlh, hsvls, hsvlv);
        Scalar hhsv = new Scalar(hsvhh, hsvhs, hsvhv);
        Core.inRange(mat, lhsv, hhsv, mask);

        // Создаём чёрно-белое выходное изображение
        Mat output = new Mat();
        // Конвертируем маску в 3-канальное изображение (чтобы было совместимо с RGB)
        Imgproc.cvtColor(mask, output, Imgproc.COLOR_GRAY2RGB);

        // Вычисляем центр масс (для логики автономного режима)
        centroid = Center(mask);
        center = (int) centroid.x;

        // Анализируем заполненность (для логики автономного режима)
        submat = mask.submat(new Rect(new Point(0, 100), new Point(320, 210)));
        fillValue = Core.sumElems(submat).val[0] / new Rect(new Point(0, 100), new Point(320, 210)).area() / 255;

        // Рисуем прямоугольник зоны анализа и центр масс (для визуализации)
        // Используем белый цвет для рисования на чёрно-белом изображении
        Imgproc.rectangle(output, new Point(0, 100), new Point(320, 210), new Scalar(255, 255, 255), 1);
        Imgproc.circle(output, centroid, 10, new Scalar(255, 255, 255), -1);

        // Освобождаем ресурсы (чтобы не было утечек памяти)
        mat.release();
        mask.release();
        submat.release();

        return output; // Возвращаем чёрно-белое изображение
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