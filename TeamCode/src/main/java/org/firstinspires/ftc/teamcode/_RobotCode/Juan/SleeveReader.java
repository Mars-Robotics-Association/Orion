package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveReader extends OpenCvPipeline {
    public void setRange(Scalar highHSV, Scalar lowHSV){
        this.highHSV = highHSV;
        this.lowHSV = lowHSV;
    }

    private Scalar highHSV = new Scalar(0, 0, 0);
    private Scalar lowHSV = new Scalar(0, 0, 0);

    double certainty = 0;

    @Override
    public Mat processFrame(Mat src) {
        Mat mask = new Mat();
        Mat hsv = new Mat();
        Mat dst = new Mat();

        Imgproc.cvtColor(src, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowHSV, highHSV, mask);

        double count = Core.countNonZero(mask);
        Size size = mask.size();
        double total = size.width * size.height;

        certainty = total / count;

        return dst;
    }

    public void joinComparisons(Mat original, Mat src1, Mat src2, Mat src3, Mat dst){
        Mat logicalDST = new Mat();

        Core.bitwise_or(src1, src2, logicalDST);
        Core.bitwise_or(logicalDST, src3, logicalDST);
        Core.bitwise_and(original, original, dst, logicalDST);
    }
}
