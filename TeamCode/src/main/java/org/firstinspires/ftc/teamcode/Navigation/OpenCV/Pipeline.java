package org.firstinspires.ftc.teamcode.Navigation.OpenCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class Pipeline extends OpenCvPipeline {
    Scalar lowbgr = new Scalar(0,100,100);
    Scalar highbgr = new Scalar(30, 255, 255);
    Mat result = new Mat();
    int lastresult = 0;
    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowbgr,highbgr, mask);
        Core.bitwise_and(input, input, last, mask);
        result = last;
        lastresult = 1;
        return last;
    }
    public int GetResults(){
        return lastresult;
    }


}