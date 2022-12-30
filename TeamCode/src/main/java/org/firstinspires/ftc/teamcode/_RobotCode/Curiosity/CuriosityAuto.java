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
import org.opencv.core.Mat;

@Autonomous(name="Curiostiy Autonomous",group="Curiosity")
@Config
public class CuriosityAuto extends LinearOpMode {

    private CuriosityBot robot;
    private boolean isRed;
    private boolean isLeft;
    private int xMultiplier;
    private EncoderActuator arm;
    private Servo gripper;
    private FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        robot = new CuriosityBot(this,null,true,true,true);
        robot.init();
        arm = robot.getPayload().arm;
        gripper = robot.getPayload().gripper;
        isRed=false;
        if(robot.getFieldSide().equals(BaseRobot.FieldSide.RED)){isRed=true;}
        isLeft=false;
        if(robot.getLrSide().equals(BaseRobot.LRSide.LEFT)){isLeft=true;}
        xMultiplier=1;
        if(isLeft){xMultiplier=-1;}

        waitForStart();
        robot.start();
        robot.getChassis().resetGyro();
        double coneSide = getConeSide(robot.camera);
        telemetry.addData("Position",coneSide);
        telemetry.update();
        robot.getChassis().rawDrive(0,1,0);
        //wait for time/color sensor
        robot.getChassis().stop();
        robot.getChassis().rawTurn(xMultiplier);
        //wait for time/color sensor
        //get block
        arm.goToPosition(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.GROUND));
        gripper.setPosition(0);
        //go to high pole
        
        //place
        //go back to blocks
        //get block
        //go back to pole
        //place


        //spot 1(green)
        if(coneSide==1) {
            //go left

        }
        //spot 2(purple)
        else if(coneSide==2){
            //stay in center

        }
        //spot 3(orange)
        else{
            //go right

        }
    }

    //move this somewhere else if it goes in a different class
    double getConeSide(Camera c) throws InterruptedException {
        Bitmap img = c.getImage();
        Mat in = c.convertBitmapToMat(c.shrinkBitmap(img,20,20));
        dash.sendImage(img);
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
