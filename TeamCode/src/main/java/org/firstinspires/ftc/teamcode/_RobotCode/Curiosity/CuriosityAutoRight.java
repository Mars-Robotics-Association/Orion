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

@Autonomous(name="*CURIOSITY AUTONOMOUS RIGHT*",group="Curiosity")
@Config
public class CuriosityAutoRight extends LinearOpMode {

    private CuriosityBot robot;
    private boolean isRed;
    private boolean isLeft;
    private int xMultiplier;
    private EncoderActuator arm;
    private Servo gripper;
    private FtcDashboard dash;

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
        isLeft=false;
        xMultiplier=1;
        if(isLeft){xMultiplier=-1;}

        waitForStart();
        robot.start();
        robot.getChassis().resetGyro();
        double coneSide = getConeSide(robot.camera);
        telemetry.addData("Position",coneSide);
        telemetry.update();
        //place
        goToPose(24,0,xMultiplier*-45,1);
        arm.goToPosition(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.HIGH));
        robot.getChassis().rawDrive(0,.1,0);
        sleep(1000);
        robot.getChassis().stop();
        robot.getPayload().toggleGripper(true);

        //get block
        goToPose(24,12*xMultiplier,90*xMultiplier,1);
        arm.goToPosition(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.GROUND)+8);
        robot.getPayload().toggleGripper(true);
        robot.getChassis().rawDrive(0,.1,0);
        sleep(1000);
        robot.getChassis().stop();
        robot.getPayload().toggleGripper(false);
        arm.goToPosition(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.HIGH));
        robot.getChassis().rawDrive(0,-.1,0);
        sleep(1000);
        robot.getChassis().stop();
        //go to high pole
        goToPose(24,0,xMultiplier*-45,1);
        //place
        //arm.goToPosition(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.HIGH));
        robot.getChassis().rawDrive(0,.1,0);
        sleep(1000);
        robot.getChassis().stop();
        robot.getPayload().toggleGripper(true);
        robot.getChassis().rawDrive(0,-.1,0);
        sleep(1000);
        robot.getChassis().stop();

        //get block
        goToPose(24,12*xMultiplier,90*xMultiplier,1);
        arm.goToPosition(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.GROUND)+6);
        robot.getPayload().toggleGripper(true);
        robot.getChassis().rawDrive(0,.1,0);
        sleep(1000);
        robot.getChassis().stop();
        robot.getPayload().toggleGripper(false);
        arm.goToPosition(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.HIGH));
        robot.getChassis().rawDrive(0,-.1,0);
        sleep(1000);
        robot.getChassis().stop();
        //place
        goToPose(24,0,xMultiplier*-45,1);
        //arm.goToPosition(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.HIGH));
        robot.getChassis().rawDrive(0,.1,0);
        sleep(1000);
        robot.getChassis().stop();
        robot.getPayload().toggleGripper(true);
        robot.getChassis().rawDrive(0,-.1,0);
        sleep(1000);
        robot.getChassis().stop();
        arm.goToPosition(CuriosityPayload.getPoleHeight(CuriosityPayload.Pole.LOW));


        //spot 1(green)
        if(coneSide==1) {
            //go to left
            goToPose(24,-12,0,1);
        }
        //spot 2(purple)
        else if(coneSide==2){
            //go to center
            goToPose(24,0,0,1);
        }
        //spot 3(orange)
        else{
            //go to right
            goToPose(24,12,0,1);
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

    //use coordinate system for input for this
    void goToPose(double x, double y, double angle, double speed) throws InterruptedException {
        while(robot.navigator.goTowardsPose(x,y,angle,speed)) {
            robot.update();
            robot.getPayload().update(0);
            telemetry.update();
        }
    }

}
