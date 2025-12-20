package org.firstinspires.ftc.teamcode.teamcode.openCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

public class BallDetector extends OpenCvPipeline {
    HSV hsv = new HSV();
    Mat mat = new Mat();
    Mat mask = new Mat();
    Mat submat = new Mat();

    Mat output = new Mat();
    public double fillValue = 0;
    public int center = 0;
    Point centroid;
    private Scalar lowerColor1 = new Scalar(70, 80, 30);
    private Scalar upperColor1 = new Scalar(90, 255, 255);
    private Scalar lowerColor2 = new Scalar(100, 40, 60);
    private Scalar upperColor2 = new Scalar(255, 140, 255);



    @Override

    public Mat processFrame(Mat input) {
        try {

            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);


            Mat mask1 = new Mat();
            Mat mask2 = new Mat();

            Core.inRange(mat, lowerColor1, upperColor1, mask1);
            Core.inRange(mat, lowerColor2, upperColor2, mask2);


            Core.bitwise_or(mask1, mask2, mask);


            Imgproc.cvtColor(mask, output, Imgproc.COLOR_GRAY2RGB);


            mask1.release();
            mask2.release();

            return output;

        } finally {
            // Гарантированное освобождение ресурсов
            mat.release();
            mask.release();
            submat.release();
        }
    }


}