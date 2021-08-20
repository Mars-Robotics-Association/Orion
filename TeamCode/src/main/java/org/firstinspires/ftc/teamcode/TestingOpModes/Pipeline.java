package org.firstinspires.ftc.teamcode.TestingOpModes;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class Pipeline extends OpenCvPipeline {
    Mat grey = new Mat();
    Mat newmat = new Mat();
    Scalar lowbgr = new Scalar(20,100,100);
    Scalar highbgr = new Scalar(30, 255, 255);
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2HSV);
        Core.inRange(grey, lowbgr,highbgr, newmat);

        return newmat;
    }
}
