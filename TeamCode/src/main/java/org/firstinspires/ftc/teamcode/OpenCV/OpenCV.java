package org.firstinspires.ftc.teamcode.OpenCV;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DashboardWebSocketServer;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.Base64Image;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.ConfidenceLevel;
import org.firstinspires.ftc.teamcode.Orion.NavModules.OpenCV.EncodedImageRecognition;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class OpenCV {
    private OpMode opmode;
    private TFObjectDetector tfod;

    public OpenCV(OpMode opMode){
        opmode = opMode;
    }

    private void getRecognitions(Bitmap bitmap) {
        if (tfod != null && bitmap != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            EncodedImageRecognition bt = new EncodedImageRecognition(bitmap);
//            String bit = gson.toJson(bt);
//            FullIMG img = new FullIMG(bit);
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {

                    Mat mf = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC3);
                    Utils.bitmapToMat(bitmap, mf);
                    //Core.flip(mf, mf, 1);
                    //Core.rotate(mf, mf, Core.ROTATE_90_COUNTERCLOCKWISE);

                    int left = Math.round(recognition.getLeft());
                    if (left < 0) left = 0;

                    int top = Math.round(recognition.getTop());
                    if (top < 0) top = 0;

                    int right = Math.round(recognition.getRight());
                    if (right > mf.width()) right = mf.width();

                    int bottom = Math.round(recognition.getBottom());
                    if (bottom > mf.height()) bottom = mf.height();

                    int dx = right - left;
                    int dy = bottom - top;
                    int area = dx*dy;

                    Log.i("OpenCVRecognitions", "image coords: " + left + ", " + top + ", " + dx + ", " + dy + "(" + right + ", " + bottom + ")");

                    Rect rectcrop = new Rect(left, top, dx, dy);

                    //opencv functions
                    Mat cropped = new Mat(mf, rectcrop);
                    Scalar lowbgr = new Scalar(0,100,100);
                    Scalar highbgr = new Scalar(30, 255, 255);
                    Mat hsv = new Mat();
                    Mat mask = new Mat();
                    Mat last = new Mat();
                    Imgproc.cvtColor(cropped, hsv, Imgproc.COLOR_RGB2HSV);
                    Core.inRange(hsv, lowbgr,highbgr, mask);
                    Core.bitwise_and(cropped, cropped, last, mask);
                    cropped = last;


                    Bitmap revised = Bitmap.createBitmap(cropped.width(), cropped.height(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(cropped, revised);

                    double pixelcount = 0.0;

                    for (int x = 0; x <revised.getWidth(); x++) {
                        for (int y = 0; y < revised.getHeight(); y++) {
                            int color = revised.getPixel(x,y);
                            int R = (color & 0xff0000) >> 16;
                            int G = (color & 0xff00) >> 8;
                            int B = color & 0xff;
                            if((R != 0)&&(G != 0)&&(B != 0)){
                                pixelcount++;
                            }
                        }
                    }
                    double percent = 100.0* (pixelcount/area);
                    opmode.telemetry.addData("percent yellow: ",percent);
                    float vconf = recognition.getConfidence();
                    boolean iscube = false;
                    if (vconf>=.5 && percent >= 50){
                        iscube = true;
                    }
                    opmode.telemetry.addData("Recognition: ",iscube);

                    opmode.telemetry.addData("left,top:  ", "%d, %d", left, top);
                    opmode.telemetry.addData("dx,dy:  ", "%d, %d", dx, dy);
                    opmode.telemetry.update();
                }
            }
        }
    }

    public Bitmap convertMatToBitMap(Mat input){
        Bitmap bmp = null;
        Mat rgb = new Mat();
        Imgproc.cvtColor(input, rgb, Imgproc.COLOR_BGR2RGB);

        try {
            bmp = Bitmap.createBitmap(rgb.cols(), rgb.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rgb, bmp);
        }
        catch (CvException e){
            opmode.telemetry.addData("Exception",e.getMessage());
        }
        return bmp;
    }
}
