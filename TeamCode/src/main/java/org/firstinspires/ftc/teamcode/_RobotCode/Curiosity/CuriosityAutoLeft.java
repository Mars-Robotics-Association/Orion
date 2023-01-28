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
import org.opencv.core.Mat;
import org.opencv.core.Rect;

@Autonomous(name = "*CURIOSITY AUTO LEFT*", group="Curiosity")
@Config
public class CuriosityAutoLeft extends LinearOpMode {

    private CuriosityBot robot;
    private boolean isRed;
    private boolean isLeft;
    private int sideMultiplier;
    private EncoderActuator arm;
    private Servo gripper;
    private FtcDashboard dash;

    public static double speed = 0.5;
    public static double coneStackTop = 6;
    public static double coneStackInterval = 1.4;
    public static double coneSide = 1;

    //IMPORTANT: ANY CHANGES MADE HERE SHOULD BE COPIED INTO CuriosityAutoRight
    //Only difference between the two autos should be the value of isLeft
    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        robot = new CuriosityBot(this,null,true,true,true);
        robot.init();
        arm = robot.getPayload().arm;
        gripper = robot.getPayload().gripper;
        isRed=false;
        if(robot.getFieldSide().equals(BaseRobot.FieldSide.RED)){isRed=true;}
        isLeft=true;
        sideMultiplier =1;
        if(isLeft){
            sideMultiplier =-1;}

        //START
        waitForStart();
        robot.start();
        resetRuntime();
        robot.getChassis().resetGyro();
        double coneSide = getConeSide(robot.camera);
        //telemetry.addData("Position",coneSide);
        telemetry.update();

        //resets arm
        robot.getPayload().toggleGripper(false);
        while(robot.getPayload().autoLevel()&&!isStopRequested()) telemetry.addLine("Resetting arm");
        robot.getPayload().stop();
        //places preload cone
        robot.getPayload().goToHeight(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.MID));
        goToPose(33,3.5*sideMultiplier,0,0.8);
        turnTo(-45*sideMultiplier, speed);
        robot.getPayload().toggleGripper(true);
        sleep(300);
        turnTo(0,speed);
        goToPose(55,0,0,1);
        goToPose(48,0,0,1);
        turnTo(90*sideMultiplier,speed);

        //picks up and places a cone from the stack
        int coneCounter = 0;
        while(coneCounter<2){
            //raises arm to pick up cone
            robot.getPayload().goToHeight(coneStackTop-(coneCounter*coneStackInterval));
            //goes to the stack
            goToPose(50.5, 24*sideMultiplier,90*sideMultiplier,speed);
            //picks up cone
            robot.getPayload().toggleGripper(false);
            sleep(500);
            //moves arm up
            robot.getPayload().goToHeight(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.MID));
            //increases cone counter, as it has taken a cone off the stack
            coneCounter ++;
            //goes to place
            goToPose(45, 3*sideMultiplier,-135*sideMultiplier,speed);
            //places cone
            robot.getPayload().toggleGripper(true);
            sleep(300);
            goToPose(48, 0,180,speed);
        }

        robot.getPayload().goToHeight(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.GROUND));

        //spot 1(green)
        if(coneSide==1) {
            //go to left
            goToPoseNoTimer(44,-20,0,1);
        }
        //spot 2(purple)
        else if(coneSide==2){
            //go to center
            goToPoseNoTimer(44,0,0,1);
        }
        //spot 3(orange)
        else{
            //go to right
            goToPoseNoTimer(44,24,-90*sideMultiplier,1);
        }
        telemetry.addLine("DONE");
        telemetry.update();
        robot.stop();
        stop();
    }

    //move this somewhere else if it goes in a different class
    double getConeSide(Camera c) throws InterruptedException {
        Bitmap img = c.getImage();
        Mat cropped = new Mat(c.convertBitmapToMat(img),new Rect(5*img.getWidth()/8,img.getHeight()/3,img.getWidth()/4,img.getHeight()/3));
        Bitmap img2=c.convertMatToBitMap(cropped);
        Mat in = c.convertBitmapToMat(c.shrinkBitmap(img2,20,20));
        dash.sendImage(img);
        Mat greenMat = c.isolateColor(in, OpenCVColors.ConeGreenH,OpenCVColors.ConeGreenL);
        Mat purpleMat = c.isolateColor(in,OpenCVColors.ConePurpleH,OpenCVColors.ConePurpleL);
        Mat orangeMat = c.isolateColor(in,OpenCVColors.ConeOrangeH,OpenCVColors.ConeOrangeL);


        int greenCount = c.countPixels(c.convertMatToBitMap(greenMat));
        int purpleCount = c.countPixels(c.convertMatToBitMap(purpleMat));
        int orangeCount = c.countPixels(c.convertMatToBitMap(orangeMat));
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

    //positive x is forward in inches, positive y is right in inches, use coordinates for x and y
    //right turn is positive angle in degrees
    void goToPose(double x, double y, double angle, double speed) throws InterruptedException {
        while(robot.navigator.goTowardsPose(x,y,angle,speed) && !isStopRequested() ) {//&& getRuntime()<28
            robot.update();
            telemetry.update();
        }
    }
    void goToPoseNoTimer(double x, double y, double angle, double speed) throws InterruptedException {
        while(robot.navigator.goTowardsPose(x,y,angle,speed) && !isStopRequested()) {
            robot.update();
            telemetry.update();
        }
    }
    void turnTo(double angle, double speed) throws InterruptedException {
        while (robot.navigator.turnTowards(angle,speed)&&getRuntime()<28){
            robot.update();
            telemetry.update();
        }
    }

}
