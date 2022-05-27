package org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCV;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCVColors;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class FreightFrenzyPipeline extends OpenCvPipeline
{

    private Mat blurInput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private Mat findContoursOutputMat = new Mat();
    private Mat finalContourOutputMat = new Mat();

    private int largestX, largestY;
    private double largestArea;

    public FreightFrenzyPipeline() {

        largestX = -1;
        largestY = -1;
        largestArea = -1;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Step Blur0 (stage 1):
        blurInput = input;
        BlurType blurType = BlurType.get("Box Blur");
        double blurRadius = 7;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0  (stage 2):
        Mat hsvThresholdInput = blurOutput;
        Scalar[] yellow = OpenCVColors.broaden(OpenCVColors.YellowL,OpenCVColors.YellowH);
        Scalar lowhsv = OpenCVColors.RedL;
        Scalar highhsv = OpenCVColors.RedH;
        hsvThreshold(hsvThresholdInput, lowhsv, highhsv, hsvThresholdOutput);
        hsvThresholdOutput=OpenCV.range(blurOutput,lowhsv,highhsv);

        // Step Find_Contours0 (stage 3):
        Mat findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = false;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
        findContoursOutputMat = input.clone();
        for(int i = 0; i < findContoursOutput.size(); i++) {
            Imgproc.drawContours(findContoursOutputMat, findContoursOutput, i, new Scalar(255, 255, 255), 2);
        }

        // Finding largest contour (stage 4):
        finalContourOutputMat = input.clone();
        largestArea = -1;
        largestX = -1;
        largestY = -1;
        int largestContourIndex = -1;
        for(int i = 0; i < findContoursOutput.size(); i++) {
            MatOfPoint contour = findContoursOutput.get(i);
            double contourArea = Imgproc.contourArea(contour);
            if(contourArea > largestArea) {
                Moments p = Imgproc.moments(contour, false);
                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());

                largestContourIndex = i;
                largestX = x;
                largestY = y;
                largestArea = contourArea;
            }
        }
//        if(largestContourIndex != -1)
//            Imgproc.drawContours(finalContourOutputMat, findContoursOutput, largestContourIndex, new Scalar(255, 255, 255), 2);
        for(int i=0;i<findContoursOutput.size();i++){
//            Imgproc.drawContours(finalContourOutputMat,findContoursOutput,i,new Scalar(255,255,255),2);
            Rect rect = Imgproc.boundingRect(findContoursOutput.get(i));
//            Mat cropped = OpenCV.crop(finalContourOutputMat,rect);
//            cropped = OpenCV.isolateColor(cropped,OpenCVColors.YellowH,OpenCVColors.YellowL);
            //double percent = OpenCV.percentColor(OpenCV.convertMatToBitMap(cropped));
            //if(percent>.7)
            if(rect.area()>input.width()*input.height()/50) {
                if ((double) rect.height / rect.width < 1.5 && (double) rect.height / rect.width > (double) 2 / 3)
                    Imgproc.rectangle(finalContourOutputMat, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 0, 255), 2);
//                else
//                    Imgproc.rectangle(finalContourOutputMat, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0), 2);
            }
        }

        // Step HSV_Threshold0  (stage 2):
        hsvThresholdInput = blurOutput;
        lowhsv = OpenCVColors.BlueL;
        highhsv = OpenCVColors.BlueH;
        hsvThreshold(hsvThresholdInput, lowhsv, highhsv, hsvThresholdOutput);
        hsvThresholdOutput=OpenCV.range(blurOutput,lowhsv,highhsv);

        // Step Find_Contours0 (stage 3):
        findContoursInput = hsvThresholdOutput;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
        findContoursOutputMat = input.clone();
        for(int i = 0; i < findContoursOutput.size(); i++) {
            Imgproc.drawContours(findContoursOutputMat, findContoursOutput, i, new Scalar(255, 255, 255), 2);
        }

        // Finding largest contour (stage 4):
        largestArea = -1;
        largestX = -1;
        largestY = -1;
        for(int i = 0; i < findContoursOutput.size(); i++) {
            MatOfPoint contour = findContoursOutput.get(i);
            double contourArea = Imgproc.contourArea(contour);
            if(contourArea > largestArea) {
                Moments p = Imgproc.moments(contour, false);
                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());

                largestContourIndex = i;
                largestX = x;
                largestY = y;
                largestArea = contourArea;
            }
        }
//        if(largestContourIndex != -1)
//            Imgproc.drawContours(finalContourOutputMat, findContoursOutput, largestContourIndex, new Scalar(255, 255, 255), 2);
        for(int i=0;i<findContoursOutput.size();i++){
//            Imgproc.drawContours(finalContourOutputMat,findContoursOutput,i,new Scalar(255,255,255),2);
            Rect rect = Imgproc.boundingRect(findContoursOutput.get(i));
//            Mat cropped = OpenCV.crop(finalContourOutputMat,rect);
//            cropped = OpenCV.isolateColor(cropped,OpenCVColors.YellowH,OpenCVColors.YellowL);
            //double percent = OpenCV.percentColor(OpenCV.convertMatToBitMap(cropped));
            //if(percent>.7)
            if(rect.area()>input.width()*input.height()/50) {
                if ((double) rect.height / rect.width < 1.5 && (double) rect.height / rect.width > (double) 2 / 3)
                    Imgproc.rectangle(finalContourOutputMat, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(255, 0, 0), 2);
//                else
//                    Imgproc.rectangle(finalContourOutputMat, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0), 2);
            }
        }

        return finalContourOutputMat;
    }

    public int[] getPosition() {
        return new int[] {largestX, largestY};
    }


    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    private void hsvThreshold(Mat input, Scalar low, Scalar high, Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, low, high, out);
    }

    private void findContours(Mat input, boolean externalOnly,
                              ArrayList<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }
}