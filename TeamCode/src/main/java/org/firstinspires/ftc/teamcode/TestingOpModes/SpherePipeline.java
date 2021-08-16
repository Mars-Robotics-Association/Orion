package org.firstinspires.ftc.teamcode.TestingOpModes;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class SpherePipeline extends OpenCvPipeline {
    Mat grey = new Mat();
    Mat newmat = new Mat();
    Scalar highbgr = new Scalar(255,255,255);
    Scalar lowbgr = new Scalar(230,230,230);
    @Override
    public Mat processFrame(Mat input) {
        Core.inRange(input, lowbgr,highbgr, newmat);

        return newmat;
    }
}
