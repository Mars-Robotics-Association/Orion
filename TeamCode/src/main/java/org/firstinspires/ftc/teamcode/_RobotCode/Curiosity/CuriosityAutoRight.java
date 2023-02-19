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

@Autonomous(name = "*CURIOSITY AUTO RIGHT*", group="Curiosity")
@Config
public class CuriosityAutoRight extends LinearOpMode {

    private CuriosityBot robot;
    private boolean isRed;
    private boolean isLeft;
    private int sideMultiplier;
    private EncoderActuator arm;
    private Servo gripper;
    private FtcDashboard dash;

    public static double speed = 1;
    public static double coneStackTop = 6;
    public static double coneStackInterval = 1.4;
    public static double coneSide = 1;

    //IMPORTANT: ANY CHANGES MADE HERE SHOULD BE COPIED INTO CuriosityAutoRight
    //Only difference between the two autos should be the value of isLeft
    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        robot = new CuriosityBot(this,null,true,true,true,false);
        robot.init();
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
        telemetry.addData("Position",coneSide);
        telemetry.update();

        //resets arm

        //places preload cone
        if(coneSide==1) {
            goToPose(8, -5, 0, 0.8);
            turnTo(-45, speed);
            deployCone(CuriosityPayload.Pole.LOW);
            //sleep(300);
            turnTo(0, speed);
        }
        goToPose(55,0,0,1);
        goToPose(48,0,0,1);
        turnTo(90,speed);

        double conePickupX = 47;
        double conePickupY = 25;

        //raises arm to pick up cone
        goToPose(conePickupX, conePickupY,90,speed);//goes to the stack
        //picks up cone
        pickUpCone(5);
        //sleep(500);
        //moves arm up
        goToPose(conePickupX,(conePickupY-10),90,speed);//backs up a bit to clear stack
        goToPose(52, -6,-45,speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.HIGH);
        //sleep(300);

        //raises arm to pick up cone
        goToPose(conePickupX, conePickupY,90,speed);//goes to the stack
        //picks up cone
        pickUpCone(4);
        //sleep(500);
        //moves arm up
        goToPose(conePickupX,(conePickupY-8),90,speed);//backs up a bit to clear stack
        goToPose(44, 10,-180,speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.LOW);
        //sleep(300);
        goToPose(48,10,-180,speed);//back up a bit
        turnTo(90,speed);

        //raises arm to pick up cone
        goToPose(conePickupX, conePickupY,90,speed);//goes to the stack
        //picks up cone
        pickUpCone(3);
        //sleep(500);
        //moves arm up
        goToPose(conePickupX,(conePickupY-10),90,speed);//backs up a bit to clear stack
        goToPose(45, -5,-135,speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.MID);
        //sleep(300);



//        robot.getPayload().goToHeight(Old_CuriosityPayload.getPoleHeight(Old_CuriosityPayload.Pole.GROUND));

        //spot 1(green)
        if(coneSide==1) {
            //go to left
            goToPoseNoLR(48,-20,-180,1);
        }
        //spot 2(purple)
        else if(coneSide==2){
            //go to center
            goToPoseNoLR(48,0,-180,1);
        }
        //spot 3(orange)
        else{
            //go to right
            goToPoseNoLR(48,24,-180,1);
        }
        telemetry.addLine("DONE");
        telemetry.update();
        robot.stop();
        stop();
    }

    //move this somewhere else if it goes in a different class
    double getConeSide(Camera c) throws InterruptedException {
        Bitmap img = c.getImage();
        Mat cropped = new Mat(c.convertBitmapToMat(img),new Rect(0*img.getWidth(),0*img.getHeight(),img.getWidth(),img.getHeight()));
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
        while(robot.navigator.goTowardsPose(x,y*sideMultiplier,angle*sideMultiplier,speed) && !isStopRequested() ) {//&& getRuntime()<28
            robot.update();
            telemetry.update();
        }
    }
    void goToPoseNoLR(double x, double y, double angle, double speed) throws InterruptedException {
        while(robot.navigator.goTowardsPose(x,y,angle*sideMultiplier,speed) && !isStopRequested()) {
            robot.update();
            telemetry.update();
        }
    }
    void turnTo(double angle, double speed) throws InterruptedException {
        while (robot.navigator.turnTowards(sideMultiplier*angle,speed)&&!isStopRequested()){
            robot.update();
            telemetry.update();
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
