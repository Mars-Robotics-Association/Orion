package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;
import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCVColors;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

@Autonomous(name = "*JUAN AUTONOMOUS RELEASED (BF)*", group="Juan_RELEASED")
@Config

public class JuanAutonomous_RELEASED_BF extends LinearOpMode {
    private Juan_RELEASED robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan_RELEASED(this,true,true,true);
        robot.init();

        //START
        waitForStart();
        robot.start();
        resetRuntime();
        robot.getChassis().resetGyro();
        double coneSide = getConeSide(robot.camera);
        int sideMultiplier =1;
        telemetry.addData("Position",coneSide);
        telemetry.update();

        if(coneSide==1) {
            //go to left
//            goToPoseNoTimer(44,-20,0,1);
        }
        //spot 2(purple)
        else if(coneSide==2){
            //go to center
//            goToPoseNoTimer(44,0,0,1);
        }
        //spot 3(orange)
        else{
            //go to right
//            goToPoseNoTimer(44,24,-90*sideMultiplier,1);
        }
        //telemetry.addLine("DONE");
        //telemetry.update();
        robot.stop();
        stop();

    }

    //move this somewhere else if it goes in a different class
    double getConeSide(Camera c) throws InterruptedException {
        Bitmap img = c.getImage();
        // original Curiosity crop was 5/8  1/3  1/4  1/3
        Mat cropped = new Mat(c.convertBitmapToMat(img), new Rect( (3 * img.getWidth() / 5) + (img.getWidth() / 10), 4 * img.getWidth() / 5, img.getHeight() / 5, img.getHeight() / 5 - 10));
        Bitmap img2 = c.convertMatToBitMap(cropped);
        Mat in = c.convertBitmapToMat(c.shrinkBitmap(img2, 20, 20));
        //dash.sendImage(img);
        Mat greenMat = c.isolateColor(in, OpenCVColors.ConeGreenH, OpenCVColors.ConeGreenL);
        Mat purpleMat = c.isolateColor(in, OpenCVColors.ConePurpleH, OpenCVColors.ConePurpleL);
        Mat orangeMat = c.isolateColor(in, OpenCVColors.ConeOrangeH, OpenCVColors.ConeOrangeL);

        int greenCount = c.countPixels(c.convertMatToBitMap(greenMat));
        int purpleCount = c.countPixels(c.convertMatToBitMap(purpleMat));
        int orangeCount = c.countPixels(c.convertMatToBitMap(orangeMat));

        telemetry.addLine("GREEN: "+ greenCount + " PURPLE: " + purpleCount + " ORANGE: " + orangeCount);
        telemetry.update();

        robot.dashboard.sendImage(c.growBitmap(c.convertMatToBitMap(greenMat),200,200));

        if(greenCount>purpleCount&&greenCount>orangeCount){
            //robot.dashboard.sendImage(c.growBitmap(c.convertMatToBitMap(greenMat),200,200));
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
        //robot.stop();
        //stop();
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

