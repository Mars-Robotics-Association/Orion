package org.firstinspires.ftc.teamcode.TestingOpModes;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCV;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines.BoundingPipe;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines.Pipeline;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class cameraTesting extends OpMode {

    Camera cam;
    FtcDashboard d;

    @Override
    public void init() {
        cam = new Camera(this,"Webcam 1",false);
        d=FtcDashboard.getInstance();
        msStuckDetectLoop=50000;
    }

    @Override
    public void loop() {
        Bitmap img;
        try {
            img = cam.GetImage();
            img = OpenCV.ShrinkBitmap(img,img.getWidth()/10,img.getHeight()/10);
            Mat m = OpenCV.convertBitmapToMat(img);
            OpenCvPipeline p = new FreightFrenzyPipeline();
            Mat m2=p.processFrame(m);
            Bitmap img2=OpenCV.convertMatToBitMap(m2);
            //img2=OpenCV.GrowBitmap(img2,img2.getWidth()*10,img2.getHeight()*10);
            d.sendImage(img2);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
