package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveReader extends OpenCvPipeline {
    enum POST_PROCESS {
        FAST,
        BEAUTIFUL
    }

    private Scalar highHSV = new Scalar(0, 0, 0);
    private Scalar lowHSV = new Scalar(0, 0, 0);

    double certainty = 0;

    Mat hsv = new Mat();
    Mat mask1 = new Mat();
    Mat mask2 = new Mat();
    Mat mask3 = new Mat();
    Mat dst = new Mat();

    @Override
    public Mat processFrame(Mat src) {
        double certainty1;
        double certainty2;
        double certainty3;
        SleeveColor PURPLE = SleeveColor.PURPLE;
        SleeveColor ORANGE = SleeveColor.ORANGE;
        SleeveColor GREEN = SleeveColor.GREEN;

        Imgproc.cvtColor(src, hsv, Imgproc.COLOR_RGB2HSV);

        Size size = hsv.size();
        double total = size.width * size.height;

        Core.inRange(hsv, PURPLE.lowColor, PURPLE.highColor, mask1);
        PURPLE.certainty =  total / Core.countNonZero(mask1);
        Core.inRange(hsv, ORANGE.lowColor, ORANGE.highColor, mask3);
        ORANGE.certainty =  total / Core.countNonZero(mask2);
        Core.inRange(hsv, GREEN.lowColor, GREEN.highColor, mask3);
        GREEN.certainty =  total / Core.countNonZero(mask3);

        if(Juan.POST_PROCESS == POST_PROCESS.FAST){
            return fastPostProcess(src, mask1, mask2, mask3);
        }else{
            return beautifulPostProcess(src, mask1, mask2, mask3);
        }
    }

    private Mat beautifulPostProcess(Mat src, Mat mask1, Mat mask2, Mat mask3){
        return src;
    }

    private Mat fastPostProcess(Mat src, Mat mask1, Mat mask2, Mat mask3){
        Mat logicalDST = new Mat();

        Core.bitwise_or(mask1, mask2, logicalDST);
        Core.bitwise_or(logicalDST, mask3, logicalDST);
        Core.bitwise_and(src, src, dst, logicalDST);

        return dst;
    }
}
