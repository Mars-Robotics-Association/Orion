package org.firstinspires.ftc.teamcode.TestingOpModes;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCV;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines.BoundingPipe;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.Pipelines.Pipeline;
import org.firstinspires.ftc.teamcode._RobotCode.Demobot2022.Demobot;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@TeleOp
public class cameraTesting extends OpMode {

    Camera cam;
    FtcDashboard d;
    Demobot demo;

    @Override
    public void init() {
        cam = new Camera(this,"Webcam 1",false);
        d=FtcDashboard.getInstance();
        msStuckDetectLoop=50000;
        demo = new Demobot(this,true,false,false);
    }

    @Override
    public void loop() {
        Bitmap img;
        try {
            img = cam.GetImage();
            img = cam.ShrinkBitmap(img,img.getWidth()/10,img.getHeight()/10);
            Mat m = cam.convertBitmapToMat(img);
            OpenCvPipeline p = new FreightFrenzyPipeline();
            m=p.processFrame(m);
            img=cam.convertMatToBitMap(m);
            //img=cam.GrowBitmap(img,img.getWidth()*10,img.getHeight()*10);
            d.sendImage(img);
            ArrayList<Rect> r = ((FreightFrenzyPipeline)p).getRects();
            if(r.size()!=0) {
                Rect rect = r.get(0);
                int rectx = (rect.x + rect.x + rect.width) / 2;
                if (rectx < img.getWidth() / 3) {
                    demo.getChassis().rawTurn(.2);
                } else if (rectx>2*img.getWidth()/3){
                    demo.getChassis().rawTurn(-.2);
                }
                else{
                    demo.getChassis().rawTurn(0);
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}