package org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines;



import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;


public class BoundingPipe extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        ColorPipe p = new ColorPipe();
        p.onViewportTapped();//blue
        p.onViewportTapped();//yellow
        //p.onViewportTapped();//white
        //p.onViewportTapped();//green
        input = p.processFrame(input);
        Mat gray = new Mat();
        Imgproc.cvtColor(input,gray,Imgproc.COLOR_RGB2GRAY);
        Mat thresh = new Mat();
        Imgproc.threshold(gray,thresh,0,255,Imgproc.THRESH_BINARY_INV+ Imgproc.THRESH_OTSU);
        Mat send = new Mat();
        Imgproc.Canny(thresh,send,100,200);
        List<MatOfPoint> cnts = new ArrayList<>();
        Mat h = new Mat();
        Imgproc.findContours(send,cnts,h,Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);
        Mat drawing = Mat.zeros(thresh.size(), CvType.CV_8UC3);
        Random r = new Random(12345);
        for(int i=0;i<cnts.size();i++){
            Scalar color = new Scalar(0,0,255);
            Imgproc.drawContours(drawing,cnts,i,color,2,Imgproc.LINE_8,h,0,new Point());
        }
        if(cnts.size()!=0) {
            Rect rect = Imgproc.boundingRect(cnts.get(0));
            double max = 0;
            for (MatOfPoint c : cnts) {
                double s = c.size().area();
                if (s > max) {
                    max = s;
                    rect = Imgproc.boundingRect(c);
                }
                //Rect rect = Imgproc.boundingRect(c);

            }
            Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 0, 255), 2);
        }
        return input;
    }
}
