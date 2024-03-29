package org.firstinspires.ftc.teamcode.TestingOpModes;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode._RobotCode.Demobot2022.Demobot;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

import java.util.ArrayList;

@TeleOp(name = "camera testing")
public class cameraTesting extends OpMode {

    Camera camera;
    FtcDashboard dash;
    Demobot demobot;
    FreightFrenzyPipeline pipeline;

    @Override
    public void init() {
        camera = new Camera(this,"Webcam 1");
        dash = FtcDashboard.getInstance();
        msStuckDetectLoop = 50000;
        demobot = new Demobot(this,true,false,false);
        pipeline = new FreightFrenzyPipeline();
        // gpu resources take time to allocate!
        // don't put that in a loop!
    }

    @Override
    public void loop() {
        Bitmap img = null;
        Mat imageMat = null;
        try {
            img = camera.getImage();
            img = camera.shrinkBitmap(img, img.getWidth() / 10,img.getHeight() / 10);
            imageMat = camera.convertBitmapToMat(img);
            imageMat = pipeline.processFrame(imageMat);
            img = camera.convertMatToBitMap(imageMat);
            //img = camera.GrowBitmap(img, img.getWidth() * 10, img.getHeight() * 10);
            dash.sendImage(img);
            ArrayList<Rect> boxRectArray = pipeline.getRects();


            if(boxRectArray.size() != 0) { // if there's no objects to track, don't track I guess?
                Rect boxRect = boxRectArray.get(0); // get the current rect

                int centerX = boxRect.x + (boxRect.width / 2); // get the center of it
                int halfOfWidth = img.getWidth() / 2; // half of width
                int dStart = img.getWidth()/3;

                telemetry.addData("centerX",centerX);
                telemetry.addData("start",dStart);
                telemetry.addData("end",2*dStart);
                if (centerX < dStart) {
                    demobot.getChassis().rawTurn(.2); // object is too far left
                    telemetry.addData("Going","Left");
                } else if (centerX > 2*dStart){
                    demobot.getChassis().rawTurn(-.2); // object is too far right
                    telemetry.addData("Going","Right");
                }
                else{
                    demobot.getChassis().rawTurn(0);
                    telemetry.addData("Going","Found");
                }
            }
            else{
                demobot.getChassis().rawTurn(0);
                telemetry.addData("Going","None");
            }
        } catch (InterruptedException e) { // stop button was pressed or we took too long
            e.printStackTrace();
        }
        telemetry.update();
        // cleanup time!
        // remembered this from working with low-level interop
        if(img != null)img.recycle();
        if(imageMat != null)imageMat.release();
    }
}