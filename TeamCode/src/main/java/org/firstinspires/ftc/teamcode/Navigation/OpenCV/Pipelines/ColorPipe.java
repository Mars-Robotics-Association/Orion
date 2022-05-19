package org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines;

import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCVColors;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorPipe extends OpenCvPipeline {
    int color = 1;
    Scalar high = OpenCVColors.RedH;
    Scalar low = OpenCVColors.RedL;
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
                high = OpenCVColors.RedH;
                low = OpenCVColors.RedL;
                break;
            case(2):
                high = OpenCVColors.BlueH;
                low = OpenCVColors.BlueL;
                break;
            case(3):
                high = OpenCVColors.YellowH;
                low = OpenCVColors.YellowL;
                break;
            case(4):
                high = OpenCVColors.WhiteH;
                low = OpenCVColors.WhiteL;
                break;
            case(5):
                high = OpenCVColors.MarsGreenH;
                low = OpenCVColors.MarsGreenL;
                break;
        }
    }

    public Mat range(Mat input)
    {
        Mat hsvMat = new Mat();
        Mat mask = new Mat();
        Imgproc.cvtColor(input,hsvMat,Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat,low,high,mask);
        return mask;
    }
}
