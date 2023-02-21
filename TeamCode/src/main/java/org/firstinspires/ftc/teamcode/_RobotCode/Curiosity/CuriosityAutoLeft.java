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
        isLeft=false;
        sideMultiplier =1;
        if(isLeft){
            sideMultiplier =-1;}

        //START
        waitForStart();
        robot.start();
        resetRuntime();
        robot.getChassis().resetGyro();
        int coneSide = getConeSide(robot.camera);
        telemetry.addData("Position",coneSide);
        telemetry.update();

        //1=stack for left, 1=far for right
        //spot 1(green)
        if(coneSide==1) {
            StackStop();
        }
        //spot 2(purple)
        else if(coneSide==2){
            CenterStop();
        }
        //spot 3(orange)
        else{
            FarStop();
        }
        telemetry.addLine("DONE");
        telemetry.update();
        robot.stop();
        stop();
    }

    enum Junction {UPPER,STACK,CENTER,FAR,LOWER};

    //move this somewhere else if it goes in a different class
    int getConeSide(Camera c) throws InterruptedException {
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

    CuriosityAutoLeft.Junction[] getOrder(int coneSide){
        switch(coneSide){
            case(1):
                return new CuriosityAutoLeft.Junction[] {CuriosityAutoLeft.Junction.LOWER,CuriosityAutoLeft.Junction.UPPER, CuriosityAutoLeft.Junction.CENTER, CuriosityAutoLeft.Junction.STACK};
            case(2):
                return new CuriosityAutoLeft.Junction[] {CuriosityAutoLeft.Junction.LOWER,CuriosityAutoLeft.Junction.STACK, CuriosityAutoLeft.Junction.UPPER, CuriosityAutoLeft.Junction.CENTER};
            case(3):
                return new CuriosityAutoLeft.Junction[] {CuriosityAutoLeft.Junction.STACK, CuriosityAutoLeft.Junction.UPPER, CuriosityAutoLeft.Junction.CENTER, CuriosityAutoLeft.Junction.FAR};
        }
        return new CuriosityAutoLeft.Junction[] {CuriosityAutoLeft.Junction.LOWER, CuriosityAutoLeft.Junction.UPPER, CuriosityAutoLeft.Junction.CENTER, CuriosityAutoLeft.Junction.STACK};
    }

    double[] getCords(CuriosityAutoLeft.Junction j){
        if(j== CuriosityAutoLeft.Junction.LOWER){
            return new double[] {8,-5,0};
        }
        else if(j== CuriosityAutoLeft.Junction.UPPER)
        {
            return new double[] {52, -6,-45};
        }
        else if(j== CuriosityAutoLeft.Junction.STACK){
            return new double[] {44, 10,-180};
        }
        else if(j== CuriosityAutoLeft.Junction.CENTER){
            return new double[] {45, -5,-135};
        }
        //FAR
        else{
            return new double[] {45,-29,-135};
        }
    }

    CuriosityPayload.Pole getHeight(CuriosityAutoLeft.Junction j){
        if(j== CuriosityAutoLeft.Junction.LOWER){
            return CuriosityPayload.Pole.LOW;
        }
        else if(j== CuriosityAutoLeft.Junction.UPPER)
        {
            return CuriosityPayload.Pole.HIGH;
        }
        else if(j== CuriosityAutoLeft.Junction.STACK){
            return CuriosityPayload.Pole.LOW;
        }
        else if(j== CuriosityAutoLeft.Junction.CENTER){
            return CuriosityPayload.Pole.MID;
        }
        //FAR
        else{
            return CuriosityPayload.Pole.HIGH;
        }
    }

    void StackStop() throws InterruptedException {
        double conePickupX = 47;
        double conePickupY = 25;
        //places preload cone
        goToPose(8,-5,0, 0.8);
        turnTo(-45, speed);
        deployCone(CuriosityPayload.Pole.LOW);
        //sleep(300);
        turnTo(0, speed);
        goToPose(55, 0, 0, 1);
        goToPose(48, 0, 0, 1);
        //raises arm to pick up cone
        turnTo(90, speed);
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
        goToPose(48, -5,-135,speed);//back up a bit

        //go to left
        goToPoseNoLR(48,-20,-180,1);
    }

    void CenterStop() throws InterruptedException {
        double conePickupX = 47;
        double conePickupY = 25;
        //places preload cone
        goToPose(8,-5,0, 0.8);
        turnTo(-45, speed);
        deployCone(CuriosityPayload.Pole.LOW);
        //sleep(300);
        turnTo(0, speed);
        goToPose(55, 0, 0, 1);
        goToPose(48, 0, 0, 1);

        //raises arm to pick up cone
        turnTo(90, speed);
        goToPose(conePickupX, conePickupY,90,speed);//goes to the stack
        //picks up cone
        pickUpCone(5);
        //sleep(500);
        //moves arm up
        goToPose(conePickupX,(conePickupY-10),90,speed);//backs up a bit to clear stack
        goToPose(45, -5,-135,speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.MID);
        //sleep(300);
        goToPose(conePickupX, conePickupY,90,speed);//goes to the stack
        //picks up cone
        pickUpCone(4);
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
        pickUpCone(3);
        //sleep(500);
        //moves arm up
        goToPose(conePickupX,(conePickupY-8),90,speed);//backs up a bit to clear stack
        goToPose(44, 10,-180,speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.LOW);
        //sleep(300);
        goToPose(48,10,-180,speed);//back up a bit
        turnTo(90,speed);

        //go to center
        goToPoseNoLR(48,0,-180,1);
    }

    void FarStop() throws InterruptedException {
        double conePickupX = 47;
        double conePickupY = 25;
        //places preload cone
        goToPose(45, -5,-135,speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.MID);
        //sleep(300);
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

        //raises arm to pick up cone
        goToPose(conePickupX, conePickupY,90,speed);//goes to the stack
        //picks up cone
        pickUpCone(3);
        //sleep(500);
        //moves arm up
        goToPose(conePickupX,(conePickupY-8),90,speed);//backs up a bit to clear stack
        goToPose(45,-29,-135,speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.HIGH);
        //sleep(300);
        goToPose(48,10,-180,speed);//back up a bit
        turnTo(90,speed);

        //go to right
        goToPoseNoLR(48,24,-180,1);
    }
}

//    double[] placeXYA = getCords(CuriosityAutoLeft.Junction.LOWER);
//    if(coneSide!=3) {
//        //places preload cone
//        goToPose(placeXYA[0], placeXYA[1], placeXYA[2], 0.8);
//        turnTo(-45, speed);
//        deployCone(CuriosityPayload.Pole.LOW);
//        //sleep(300);
//        turnTo(0, speed);
//        goToPose(55, 0, 0, 1);
//        goToPose(48, 0, 0, 1);
//        turnTo(90, speed);
//    }
//    double conePickupX = 47;
//    double conePickupY = 25;
//    CuriosityAutoLeft.Junction[] order = getOrder(coneSide);
//    for(int i=0;i<order.length;i++) {
//        placeXYA = getCords(order[i]);
//        if (!(i==0&&coneSide==3)){
//            goToPose(conePickupX, conePickupY, 90, speed);//goes to the stack
//            //picks up cone
//            pickUpCone(5 - i);
//            //sleep(500);
//            //moves arm up
//            goToPose(conePickupX, (conePickupY - 10), 90, speed);//backs up a bit to clear stack
//        }
//        goToPose(placeXYA[0], placeXYA[1],placeXYA[2],speed);//goes to place
//        //places cone
//        deployCone(getHeight(order[i]));
//        //sleep(300);
//    }