package org.firstinspires.ftc.teamcode.TestingOpModes;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines.FreightFrenzyPipeline;
import org.opencv.core.Mat;

@TeleOp
public class cameraTesting extends OpMode {

    Camera cam;
    FtcDashboard d;

    @Override
    public void init() {
        cam = new Camera(this,"Webcam 1");
        d=FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        Bitmap img;
        try {
            img = cam.GetImage();
            Mat m = cam.convertBitmapToMat(img);
            FreightFrenzyPipeline p = new FreightFrenzyPipeline();
            Mat m2=p.processFrame(m);
            Bitmap img2=cam.convertMatToBitMap(m2);
            d.sendImage(img2);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
