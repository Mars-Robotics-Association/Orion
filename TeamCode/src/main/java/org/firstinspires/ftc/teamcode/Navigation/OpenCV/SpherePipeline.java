package org.firstinspires.ftc.teamcode.Navigation.OpenCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class SpherePipeline extends OpenCvPipeline {
    Scalar highbgr = new Scalar(255,255,255);
    Scalar lowbgr = new Scalar(230,230,230);
    @Override
    public Mat processFrame(Mat input) {
        Mat grey = new Mat();
        Mat thing = new Mat();
        Mat newmat = new Mat();
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(grey, thing, Imgproc.COLOR_HSV2RGB);
        Core.inRange(thing, lowbgr,highbgr, newmat);

        return newmat;
    }
}
