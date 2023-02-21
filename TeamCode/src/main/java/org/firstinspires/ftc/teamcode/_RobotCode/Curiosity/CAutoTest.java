package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCVColors;
import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.CuriosityBot;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

@Autonomous(name="CAutoTest",group="Curiosity")
@Config
public class CAutoTest extends LinearOpMode {

    private CuriosityBot robot;
    private FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        robot = new CuriosityBot(this,null,true,true,true,true);
        robot.init();

        waitForStart();
        robot.start();
        pickUpCone(5);
        deployCone(CuriosityPayload.Pole.HIGH);


    }

    //move this somewhere else if it goes in a different class
    double getConeSide(Camera c) throws InterruptedException {
        Bitmap img = c.getImage();
        Mat cropped = new Mat(c.convertBitmapToMat(img),new Rect(7*img.getWidth()/24,img.getHeight()/4,img.getWidth()/6,img.getHeight()/4));
        Bitmap img2=c.convertMatToBitMap(cropped);
        Mat in = c.convertBitmapToMat(c.shrinkBitmap(img2,20,20));
        dash.sendImage(img);
        Mat greenMat = c.isolateColor(in, OpenCVColors.ConeGreenH,OpenCVColors.ConeGreenL);
        Mat purpleMat = c.isolateColor(in,OpenCVColors.ConePurpleH,OpenCVColors.ConePurpleL);
        Mat orangeMat = c.isolateColor(in,OpenCVColors.ConeOrangeH,OpenCVColors.ConeOrangeL);


        int greenCount = c.countPixels(c.convertMatToBitMap(greenMat));
        int purpleCount = c.countPixels(c.convertMatToBitMap(purpleMat));
        int orangeCount = c.countPixels(c.convertMatToBitMap(orangeMat));
        dash.sendImage(img2);
        //1 is green, 2 is purple, 3 is orange
        if(greenCount>purpleCount&&greenCount>orangeCount){
            dash.sendImage(c.growBitmap(c.convertMatToBitMap(greenMat),200,200));
            return 1;}
        else if(purpleCount>greenCount&&purpleCount>orangeCount){
            dash.sendImage(c.growBitmap(c.convertMatToBitMap(purpleMat),200,200));
            return 2;
        }
        else{
            dash.sendImage(c.growBitmap(c.convertMatToBitMap(orangeMat),200,200));
            return 3;
        }
    }


    void deployCone(CuriosityPayload.Pole p){
        robot.getPayload().update(robot.getPayload().getPolePose(p)[0],robot.getPayload().getPolePose(p)[1]);
        robot.getPayload().levelGripper();
        robot.getPayload().toggleGripper(true);
        robot.getPayload().update(robot.getPayload().pickupPose[0]+6,robot.getPayload().pickupPose[1]);
    }

    void pickUpCone(int numCones){
        robot.getPayload().update(robot.getPayload().pickupPose[0]+(numCones),robot.getPayload().pickupPose[1]);
        robot.getPayload().toggleGripper(false);
        robot.getPayload().update(robot.getPayload().pickupPose[0]+6,robot.getPayload().pickupPose[1]);
    }

}
