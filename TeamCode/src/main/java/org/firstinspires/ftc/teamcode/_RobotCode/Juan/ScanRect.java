package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ScanRect extends OpenCvPipeline {
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;

    Scalar detectedColor = new Scalar(0, 0, 0);

    @Override
    public Mat processFrame(Mat input) {
        this.input = input;

        Scalar highhsv = new Scalar(12,255,167);
        Scalar lowhsv = new Scalar(0,80,71);
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowhsv,highhsv, mask);
        Core.bitwise_and(input, input, last, mask);

        lastResult = last;
        return last;
    }
    //TEMP
    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat last = new Mat();

    Mat input = null;

    Mat lastResult = null;

    public boolean detectInRange(Scalar upper, Scalar lower){
        Mat hsv = new Mat();
        Imgproc.cvtColor(lastResult, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar sum = Core.sumElems(hsv);
        long total = hsv.total();
        return true;
    }
}
