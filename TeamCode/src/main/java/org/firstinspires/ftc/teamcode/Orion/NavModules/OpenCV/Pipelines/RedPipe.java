package org.firstinspires.ftc.teamcode.Orion.NavModules.OpenCV.Pipelines;

import org.firstinspires.ftc.teamcode.Orion.NavModules.OpenCV.Pipelines.TESTFOLDERFORATEST.OpenCVColors2;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedPipe extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Mat hsvMat = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input,hsvMat,Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, OpenCVColors2.RedL, OpenCVColors2.RedH,mask);
        Core.bitwise_and(input,input,last,mask);
        return last;
    }
}
