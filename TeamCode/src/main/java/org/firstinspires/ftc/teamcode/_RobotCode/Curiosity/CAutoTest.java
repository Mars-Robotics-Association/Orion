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


    public static double coneStackTop = 6;
    public static double coneStackInterval = 1.4;
    public static double coneSide = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        robot = new CuriosityBot(this,null,true,true,true,true);
        robot.init();

        getConeSide(robot.camera);


    }

    //move this somewhere else if it goes in a different class
    double getConeSide(Camera c) throws InterruptedException {
        Bitmap img = c.getImage();
        Mat cropped = new Mat(c.convertBitmapToMat(img),new Rect(7*img.getWidth()/24,img.getHeight()/8,img.getWidth()/6,img.getHeight()/4));
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
            //dash.sendImage(c.growBitmap(c.convertMatToBitMap(greenMat),200,200));
            return 1;}
        else if(purpleCount>greenCount&&purpleCount>orangeCount){
            //dash.sendImage(c.growBitmap(c.convertMatToBitMap(purpleMat),200,200));
            return 2;
        }
        else{
            //dash.sendImage(c.growBitmap(c.convertMatToBitMap(orangeMat),200,200));
            return 3;
        }
    }


    void deployCone(CuriosityPayload.Pole p){
        //moves the arm and lift up
        double[] polePose = robot.getPayload().getPolePose(p);
        robot.getPayload().lift.goToPosition(polePose[0]);
        robot.getPayload().arm.goToPosition(polePose[1]);
        while (Math.abs(robot.getPayload().lift.getPosition()-polePose[0])>0.4
                || Math.abs(robot.getPayload().arm.getPosition()-polePose[1])>5) {
            telemetry.addLine("Lifting 2");
            telemetry.update();
            robot.getPayload().levelGripper();}

        robot.getPayload().toggleGripper(true);
        sleep(500);
    }

    void pickUpCone(int numCones){
        robot.getPayload().toggleGripper(true);
        robot.getPayload().lift.goToPosition(numCones*coneStackInterval);
        robot.getPayload().arm.goToPosition(robot.getPayload().pickupPose[1]);
        while (Math.abs(robot.getPayload().lift.getPosition()-(numCones*coneStackInterval))>0.4
                || Math.abs(robot.getPayload().arm.getPosition()-robot.getPayload().pickupPose[1])>5) {
            telemetry.addLine("Aligning");
            telemetry.update();
            robot.getPayload().levelGripper();}

        robot.getPayload().toggleGripper(false);
        sleep(1000);

        robot.getPayload().lift.goToPosition(robot.getPayload().pickupPose[0]+(numCones*coneStackInterval));
        while (Math.abs(robot.getPayload().lift.getPosition()-(robot.getPayload().pickupPose[0]+(numCones*coneStackInterval)-2))>0.4) {
            telemetry.addLine("Lifting 1");
            telemetry.update();
            robot.getPayload().levelGripper();}
    }

}
