package org.firstinspires.ftc.teamcode.Orion.NavModules.OpenCV.Pipelines;

import org.firstinspires.ftc.teamcode.Orion.NavModules.OpenCV.Pipelines.TESTFOLDERFORATEST.OpenCVColors2;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorPipe extends OpenCvPipeline {
    int color = 1;
    Scalar high = OpenCVColors2.RedH;
    Scalar low = OpenCVColors2.RedL;
    @Override
    public Mat processFrame(Mat input) {
        Mat hsvMat = new Mat();
        Mat mask = new Mat();
        Mat last = new Mat();
        Imgproc.cvtColor(input,hsvMat,Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat,low,high,mask);
        Core.bitwise_and(input,input,last,mask);
        return last;
    }

    public void onViewportTapped()
    {
        color++;
        if(color==6)
        {
            color = 1;
        }
        switch(color)
        {
            case(1):
                high = OpenCVColors2.RedH;
                low = OpenCVColors2.RedL;
                break;
            case(2):
                high = OpenCVColors2.BlueH;
                low = OpenCVColors2.BlueL;
                break;
            case(3):
                high = OpenCVColors2.YellowH;
                low = OpenCVColors2.YellowL;
                break;
            case(4):
                high = OpenCVColors2.WhiteH;
                low = OpenCVColors2.WhiteL;
                break;
            case(5):
                high = OpenCVColors2.MarsGreenH;
                low = OpenCVColors2.MarsGreenL;
                break;
        }
    }
}
