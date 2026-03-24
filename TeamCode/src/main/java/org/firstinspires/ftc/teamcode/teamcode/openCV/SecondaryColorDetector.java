package org.firstinspires.ftc.teamcode.teamcode.openCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

public class SecondaryColorDetector extends OpenCvPipeline {
    // Результаты детекции (публичные для прямого доступа)
    public double fill1 = 0;
    public int centerX1 = 0;
    public double fill2 = 0;
    public int centerX2 = 0;

    // Рабочие матрицы
    private Mat hsv = new Mat();
    private Mat mask1 = new Mat();
    private Mat mask2 = new Mat();
    private Mat output = new Mat();

    // Зона анализа (можно настроить)
    private final Rect zone = new Rect(new Point(0, 0), new Point(320, 240));

    @Override
    public Mat processFrame(Mat input) {
        // Конвертация в HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Красный цвет (первый детектор)
        Core.inRange(hsv, new Scalar(0, 180, 180), new Scalar(20, 200, 210), mask1);

        // Синий цвет (второй детектор)
        Core.inRange(hsv, new Scalar(10, 100, 100), new Scalar(60, 255, 160), mask2);

        // Анализ первого цвета
        Moments m1 = Imgproc.moments(mask1);
        centerX1 = (int)(m1.get_m10() / (m1.get_m00() + 1e-5));
        Mat roi1 = mask1.submat(zone);
        fill1 = Core.sumElems(roi1).val[0] / (255 * zone.area());

        // Анализ второго цвета
        Moments m2 = Imgproc.moments(mask2);
        centerX2 = (int)(m2.get_m10() / (m2.get_m00() + 1e-5));
        Mat roi2 = mask2.submat(zone);
        fill2 = Core.sumElems(roi2).val[0] / (255 * zone.area());

        // Визуализация
        input.copyTo(output);
        Imgproc.rectangle(output, zone.tl(), zone.br(), new Scalar(0, 255, 0), 2);
        if (fill1 > 0.000001) {
            Imgproc.circle(output, new Point(centerX1, zone.y + zone.height/2),
                    10, new Scalar(255, 0, 0), -1);
        }
        if (fill2 > 0.000001) {
            Imgproc.circle(output, new Point(centerX2, zone.y + zone.height/2),
                    10, new Scalar(0, 0, 255), -1);
        }

        return output;
    }
}