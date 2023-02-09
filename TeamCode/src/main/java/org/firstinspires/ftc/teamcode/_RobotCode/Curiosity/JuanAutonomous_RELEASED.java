package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;
import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCVColors;
import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.CuriosityBot;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

@Autonomous(name = "*Juan autonomous but curiostiy test*", group="Opportunity")
@Config

public class JuanAutonomous_RELEASED extends LinearOpMode {
    private CuriosityBot robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CuriosityBot(this,null,true,true,true);
        robot.init();

        //START
        waitForStart();
        robot.start();
        resetRuntime();
        robot.getChassis().resetGyro();
        double coneSide = getConeSide(new Camera(this,"Webcam 1"));
        int sideMultiplier =1;
        telemetry.addData("Position",coneSide);
        telemetry.update();

        if(coneSide==1) {
            //go to left
            //goToPoseNoTimer(44,-20,0,1);
        }
        //spot 2(purple)
        else if(coneSide==2){
            //go to center
            //goToPoseNoTimer(44,0,0,1);
        }
        //spot 3(orange)
        else{
            //go to right
            //goToPoseNoTimer(44,24,-90*sideMultiplier,1);
        }
        telemetry.addLine("DONE");
        telemetry.update();
        robot.stop();
        stop();

    }

    //move this somewhere else if it goes in a different class
    double getConeSide(Camera c) throws InterruptedException {
        Bitmap img = c.getImage();
        Mat cropped = new Mat(c.convertBitmapToMat(img), new Rect(5 * img.getWidth() / 8, img.getHeight() / 3, img.getWidth() / 4, img.getHeight() / 3));
        Bitmap img2 = c.convertMatToBitMap(cropped);
        Mat in = c.convertBitmapToMat(c.shrinkBitmap(img2, 20, 20));
        //dash.sendImage(img);
        Mat greenMat = c.isolateColor(in, OpenCVColors.ConeGreenH, OpenCVColors.ConeGreenL);
        Mat purpleMat = c.isolateColor(in, OpenCVColors.ConePurpleH, OpenCVColors.ConePurpleL);
        Mat orangeMat = c.isolateColor(in, OpenCVColors.ConeOrangeH, OpenCVColors.ConeOrangeL);

        int greenCount = c.countPixels(c.convertMatToBitMap(greenMat));
        int purpleCount = c.countPixels(c.convertMatToBitMap(purpleMat));
        int orangeCount = c.countPixels(c.convertMatToBitMap(orangeMat));

        if(greenCount>purpleCount&&greenCount>orangeCount){
            //dash.sendImage(c.growBitmap(c.convertMatToBitMap(greenMat),200,200));
            return 1;}
        else if(purpleCount>greenCount&&purpleCount>orangeCount){
            //dash.sendImage(c.growBitmap(c.convertMatToBitMap(purpleMat),200,200));
            return 2;
        }
        else{
           // dash.sendImage(c.growBitmap(c.convertMatToBitMap(orangeMat),200,200));
            return 3;
        }
        //telemetry.addLine("DONE");
        //telemetry.update();

        }

        //positive x is forward in inches, positive y is right in inches, use coordinates for x and y
        //right turn is positive angle in degrees

    void goToPoseNoTimer(double x, double y, double angle, double speed) throws InterruptedException {
        while(robot.navigator.goTowardsPose(x,y,angle,speed) && !isStopRequested()) {
            robot.update();
            telemetry.update();
        }

        }

    }

