package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCVColors;
import org.opencv.core.Mat;

@Autonomous(name="Curiostiy Autonomous",group="Curiosity")
@Config
public class CuriosityAuto extends LinearOpMode {

    private CuriosityBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CuriosityBot(this,null,true,true,true);
        robot.init();

        waitForStart();
        robot.start();
        robot.getChassis().resetGyro();
        double coneSide = getConeSide(robot.camera);


        if(coneSide==1) {

        }
        else if(coneSide==2){

        }
        else{

        }
    }

    //move this somewhere else if it goes in a different class
    double getConeSide(Camera c) throws InterruptedException {
        Mat in = c.convertBitmapToMat(c.shrinkBitmap(c.getImage(),20,20));
        Mat greenMat = c.isolateColor(in, OpenCVColors.ConeGreenH,OpenCVColors.ConeGreenL);
        Mat purpleMat = c.isolateColor(in,OpenCVColors.ConePurpleH,OpenCVColors.ConePurpleL);
        Mat orangeMat = c.isolateColor(in,OpenCVColors.ConeOrangeH,OpenCVColors.ConeOrangeL);
        int greenCount = c.countPixels(c.convertMatToBitMap(greenMat));
        int purpleCount = c.countPixels(c.convertMatToBitMap(purpleMat));
        int orangeCount = c.countPixels(c.convertMatToBitMap(orangeMat));
        //1 is green, 2 is purple, 3 is orange
        if(greenCount>purpleCount&&greenCount>orangeCount){return 1;}
        else if(purpleCount>greenCount&&purpleCount>orangeCount){return 2;}
        else{return 3;}
    }
}
