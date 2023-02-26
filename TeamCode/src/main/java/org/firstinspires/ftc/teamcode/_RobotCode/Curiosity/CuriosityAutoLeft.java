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
    public static double coneStackInterval = 1;
    public static double coneSide = 1;

    double conePickupX = 49;
    double conePickupY = 24;

    int conesInStack = 5;

    int lightNum=0;

    //boolean linedUpToDrop = false;

    //IMPORTANT: ANY CHANGES MADE HERE SHOULD BE COPIED INTO CuriosityAutoRight
    //Only difference between the two autos should be the value of isLeft
    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        robot = new CuriosityBot(this, null, true, true, true, true);
        robot.init();
        isRed = false;
        if (robot.getFieldSide().equals(BaseRobot.FieldSide.RED)) {
            isRed = true;
        }
        isLeft = true;
        sideMultiplier = 1;
        if (isLeft) {
            sideMultiplier = -1;
            conePickupX+=1;
        }

        //START
        waitForStart();
        robot.start();
        resetRuntime();
        robot.getChassis().resetGyro();
        robot.getPayload().toggleGripper(false);
        nextLights();
        int coneSide = getConeSide(robot.camera);
        nextLights();
        moveConeToPlace(CuriosityPayload.Pole.LOW);
        nextLights();
        telemetry.addData("Position", coneSide);
        telemetry.update();

        //places preload cone
        goToPose(5, -2, 0, 0.8);
        nextLights();
        turnTo(-45, speed);
        nextLights();
        deployCone(CuriosityPayload.Pole.LOW);
        nextLights();
        //sleep(300);
        goToPose(5, 0, 0, 0.8);
        goToPose(55, 0, 0, 1);
        goToPose(48, 0, 0, 1);

        //1=stack for left, 1=far for right
        //spot 1(green)
        if ((coneSide == 1 && isLeft) || (coneSide == 3 && !isLeft)) {
            TwoHigh();
            //TwoMid();
            //OneLow();
            //go to left
            goToPoseNoLR(48, 24, 0, 1);
        }
        //spot 2(purple)
        else if (coneSide == 2) {
            //OneLow();
            TwoHigh();
            //TwoMid();
            //go to center
            goToPoseNoLR(48, 0, 0, 1);
        }
        //spot 3(orange)
        else {
            //TwoHigh();
            //TwoMid();
            ThreeHigh();
            //go to right
            goToPoseNoLR(48, -20, 0, 1);
        }
        telemetry.addLine("DONE");
        telemetry.update();
        robot.stop();
        stop();
    }

    void OneLow() throws InterruptedException {
        moveArmToPickup(conesInStack);
        turnTo(90, speed);
        //raises arm to pick up cone
        goToPose(conePickupX, conePickupY, 90, .6);//goes to the stack
        //picks up cone
        pickUpCone(conesInStack);
        conesInStack --;
        //sleep(500);
        //moves arm up
        moveConeToPlace(CuriosityPayload.Pole.LOW);
        goToPose(conePickupX, (conePickupY - 8), 90, speed);//backs up a bit to clear stack
        goToPose(46, 10, -180, speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.LOW);
        //sleep(300);
        goToPose(48, 10, -180, speed);//back up a bit
    }

    void TwoMid() throws InterruptedException {
        moveArmToPickup(conesInStack);
        turnTo(90, speed);
        //raises arm to pick up cone
        goToPose(conePickupX, conePickupY, 90, .6);//goes to the stack
        //picks up cone
        pickUpCone(conesInStack);
        conesInStack --;
        //sleep(500);
        //moves arm up
        moveConeToPlace(CuriosityPayload.Pole.MID);
        goToPose(conePickupX, (conePickupY - 8), 90, speed);//backs up a bit to clear stack
        goToPose(47, -7, -135, speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.MID);
        //sleep(300);
        goToPose(48, -5, -135, speed);//back up a bit
    }

    void TwoHigh() throws InterruptedException {
        moveArmToPickup(conesInStack);
        turnTo(90, speed);
        //raises arm to pick up cone
        goToPose(conePickupX, conePickupY, 90, .6);//goes to the stack
        //picks up cone
        pickUpCone(conesInStack);
        conesInStack --;
        //sleep(500);
        //moves arm up
        moveConeToPlace(CuriosityPayload.Pole.HIGH);
        goToPose(conePickupX, (conePickupY - 8), 90, speed);//backs up a bit to clear stack
        goToPose(48, -4, -45, speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.HIGH);
        //sleep(300);
        goToPose(48, 6, 0, speed);//back up a bit
    }

    void ThreeHigh() throws InterruptedException {
        moveArmToPickup(conesInStack);
        turnTo(90, speed);
        //raises arm to pick up cone
        goToPose(conePickupX, conePickupY, 90, .6);//goes to the stack
        //picks up cone
        pickUpCone(conesInStack);
        conesInStack --;
        //sleep(500);
        //moves arm up
        moveConeToPlace(CuriosityPayload.Pole.HIGH);
        goToPose(conePickupX, (conePickupY - 8), 90, speed);//backs up a bit to clear stack
        goToPose(45, -29, -135, speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.LOW);
        //sleep(300);
        goToPose(48, -29, -135, speed);//back up a bit
    }


    //move this somewhere else if it goes in a different class
    int getConeSide(Camera c) throws InterruptedException {
        Bitmap img = c.getImage();
        Mat cropped = new Mat(c.convertBitmapToMat(img), new Rect(7*img.getWidth()/24,img.getHeight()/8,img.getWidth()/6,img.getHeight()/4));
        Bitmap img2 = c.convertMatToBitMap(cropped);
        Mat in = c.convertBitmapToMat(c.shrinkBitmap(img2, 20, 20));
        dash.sendImage(img);
        Mat greenMat = c.isolateColor(in, OpenCVColors.ConeGreenH, OpenCVColors.ConeGreenL);
        Mat purpleMat = c.isolateColor(in, OpenCVColors.ConePurpleH, OpenCVColors.ConePurpleL);
        Mat orangeMat = c.isolateColor(in, OpenCVColors.ConeOrangeH, OpenCVColors.ConeOrangeL);


        int greenCount = c.countPixels(c.convertMatToBitMap(greenMat));
        int purpleCount = c.countPixels(c.convertMatToBitMap(purpleMat));
        int orangeCount = c.countPixels(c.convertMatToBitMap(orangeMat));
        dash.sendImage(img2);
        //1 is green, 2 is purple, 3 is orange
        if (greenCount > purpleCount && greenCount > orangeCount) {
            dash.sendImage(c.growBitmap(c.convertMatToBitMap(greenMat), 200, 200));
            return 1;
        } else if (purpleCount > greenCount && purpleCount > orangeCount) {
            dash.sendImage(c.growBitmap(c.convertMatToBitMap(purpleMat), 200, 200));
            return 2;
        } else {
            dash.sendImage(c.growBitmap(c.convertMatToBitMap(orangeMat), 200, 200));
            return 3;
        }
    }

    //positive x is forward in inches, positive y is right in inches, use coordinates for x and y
    //right turn is positive angle in degrees
    void goToPose(double x, double y, double angle, double speed) throws InterruptedException {
        while (robot.navigator.goTowardsPose(x, y * sideMultiplier, angle * sideMultiplier, speed) && !isStopRequested()) {//&& getRuntime()<28
            robot.update();
            robot.getPayload().levelGripper();
            telemetry.update();
        }
    }

    void goToPoseNoLR(double x, double y, double angle, double speed) throws InterruptedException {
        while (robot.navigator.goTowardsPose(x, y, angle * sideMultiplier, speed) && !isStopRequested()) {
            robot.update();
            robot.getPayload().levelGripper();
            telemetry.update();
        }
    }

    void turnTo(double angle, double speed) throws InterruptedException {
        while (robot.navigator.turnTowards(sideMultiplier * angle, speed) && !isStopRequested()) {
            robot.update();
            robot.getPayload().levelGripper();
            telemetry.update();
        }
    }

    void moveConeToPlace(CuriosityPayload.Pole p){
        //moves the arm and lift up
        double[] polePose = robot.getPayload().getPolePose(p);
        robot.getPayload().lift.goToPosition(polePose[0]);
        robot.getPayload().arm.goToPosition(polePose[1]);
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

    void moveArmToPickup(int numCones){
        robot.getPayload().toggleGripper(true);
        robot.getPayload().lift.goToPosition(robot.getPayload().pickupPose[0]+(numCones*coneStackInterval)-1);
        robot.getPayload().arm.goToPosition(robot.getPayload().pickupPose[1]);
    }

    void pickUpCone(int numCones){
        robot.getPayload().toggleGripper(true);
        robot.getPayload().lift.goToPosition(numCones*coneStackInterval-1);
        robot.getPayload().arm.goToPosition(robot.getPayload().pickupPose[1]);
        while (Math.abs(robot.getPayload().lift.getPosition()-(numCones*coneStackInterval))>0.4
                || Math.abs(robot.getPayload().arm.getPosition()-robot.getPayload().pickupPose[1])>5) {
            telemetry.addLine("Aligning");
            telemetry.update();
            robot.getPayload().levelGripper();}

        robot.getPayload().toggleGripper(false);
        sleep(500);

        robot.getPayload().lift.goToPosition(robot.getPayload().pickupPose[0]+(numCones*coneStackInterval));
        while (Math.abs(robot.getPayload().lift.getPosition()-(robot.getPayload().pickupPose[0]+(numCones*coneStackInterval)-2))>0.4) {
            telemetry.addLine("Lifting 1");
            telemetry.update();
            robot.getPayload().levelGripper();}
    }


    void nextLights(){
        if(lightNum==0){
            robot.getPayload().lights.red();
        }
        else if(lightNum==1){
            robot.getPayload().lights.green();
        }
        else if(lightNum==2){
            robot.getPayload().lights.yellow();
        }
        else if(lightNum==3){
            robot.getPayload().lights.purple();
        }
        else if(lightNum==4){
            robot.getPayload().lights.blue();
        }
        else
        {
            robot.getPayload().lights.lime();
        }
        lightNum++;
        if(lightNum>5){
            lightNum=0;
        }
    }
}