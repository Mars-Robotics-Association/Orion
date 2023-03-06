package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Navigation.OpenCV.OpenCVColors;
import org.firstinspires.ftc.teamcode.Navigation.UniversalThreeWheelNavigator;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.firstinspires.ftc.teamcode.Navigation.UniversalThreeWheelNavigator.Nav_Axis;

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

    double conePickupX = 47;
    double conePickupY = 25.5;

    int conesInStack = 5;

    int lightNum=0;

    public static double pickupTime = 4;
    double pickupTimer = 4;
    double lastRuntime = 0;

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
        int coneSide = getConeSide(robot.camera);
        moveConeToPlace(CuriosityPayload.Pole.LOW);
        telemetry.addData("Position", coneSide);
        telemetry.update();

        //places preload cone on high junction
        moveConeToPlace(CuriosityPayload.Pole.HIGH);
        goToPoseOvershoot(50, 0, 0, 1,Nav_Axis.X);
        goToPoseOvershoot(55, 0, 0, 1,Nav_Axis.X);
        sleep(500);
        goToPose(44, 0, 0, 1);
        goToPose(44, -12, 0, speed);//goes to place
        //turnTo(-40,0.6);
        //places cone
        deployCone(CuriosityPayload.Pole.HIGH);
        goToPose(48,3,0,speed);

        TwoMid();
        //goToPose(47,-3,-45,speed);
        nextLights();
        robot.getPayload().lift.goToPosition(0);
        robot.getPayload().arm.goToPosition(0);
        //turnTo(-90,speed);

        //1=stack for left, 1=far for right
        //spot 1(green)
        if ((coneSide == 1 && isLeft) || (coneSide == 3 && !isLeft)) {
            goToPoseNoLR(48, -20, -90, 1);
//            moveArmToPickup(2);

            goToPoseNoLR(30, -20, 0, 1);
        }
        //spot 2(purple)
        else if (coneSide == 2) {
            goToPoseNoLR(48, 0, -90, 1);
//            moveArmToPickup(2);

            goToPoseNoLR(30, 0, 0, 1);
        }
        //spot 3(orange)
        else {
            goToPoseNoLR(48, 24, -90, 1);
//            moveArmToPickup(2);

            goToPoseNoLR(30, 24, 0, 1);
        }
        while(!isStopRequested()){robot.getPayload().levelGripper();}
        telemetry.addLine("DONE");
        telemetry.update();
        robot.stop();
        robot.getChassis().stop();
        stop();
    }

    void OneLow() throws InterruptedException {
        moveArmToPickup(conesInStack);
        turnTo(90, speed);
        //raises arm to pick up cone
        goToStack();
        //picks up cone
        pickUpCone(conesInStack);
        conesInStack --;
        //sleep(500);
        //moves arm up
        goToPose(conePickupX, (conePickupY - 12), 90, speed);//backs up a bit to clear stack
        moveConeToPlace(CuriosityPayload.Pole.LOW);
        goToPose(48, 10, -180, speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.LOW);
        //sleep(300);
        goToPose(48, 10, -180, speed);//back up a bit
    }

    void TwoMid() throws InterruptedException {
        moveArmToPickup(conesInStack);
        turnTo(90, speed);
        //raises arm to pick up cone
        goToStack();
        //picks up cone
        pickUpCone(conesInStack);
        conesInStack --;
        //sleep(500);
        //moves arm up
        goToPoseOvershoot(conePickupX, (conePickupY - 12), 90, speed, Nav_Axis.Y);//backs up a bit to clear stack
        moveConeToPlace(CuriosityPayload.Pole.MID);
        goToPose(47, -6, 90, speed);//goes to place
        turnTo(-135,0.6);
        //places cone
        deployCone(CuriosityPayload.Pole.MID);
        //sleep(300);
        //goToPose(48, -5, -135, speed);//back up a bit
    }

    void TwoHigh() throws InterruptedException {
        moveArmToPickup(conesInStack);
        turnTo(90, speed);
        //raises arm to pick up cone
        goToStack();
        //picks up cone
        pickUpCone(conesInStack);
        conesInStack --;
        //sleep(500);
        //moves arm up
        goToPose(conePickupX, (conePickupY - 12), 90, speed);//backs up a bit to clear stack
        moveConeToPlace(CuriosityPayload.Pole.HIGH);
        goToPose(47.5, -3.5, -45, speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.HIGH);
        //sleep(300);
        //goToPose(48, 6, 0, speed);//back up a bit
    }

    void ThreeHigh() throws InterruptedException {
        moveArmToPickup(conesInStack);
        turnTo(90, speed);
        //raises arm to pick up cone
        goToStack();
        //picks up cone
        pickUpCone(conesInStack);
        conesInStack --;
        //sleep(500);
        //moves arm up
        goToPose(conePickupX, (conePickupY - 12), 90, speed);//backs up a bit to clear stack
        moveConeToPlace(CuriosityPayload.Pole.HIGH);
        goToPose(45, -29, -135, speed);//goes to place
        //places cone
        deployCone(CuriosityPayload.Pole.LOW);
        //sleep(300);
        goToPose(48, -29, -135, speed);//back up a bit
    }

    public void goToStack() throws InterruptedException {
        //goToPose(conePickupX, conePickupY, 90, .6);
        lastRuntime = getRuntime();
        pickupTimer = pickupTime;
        while (robot.navigator.goTowardsPose(conePickupX, conePickupY*sideMultiplier, 90*sideMultiplier, .6) && pickupTimer>0 && !isStopRequested()) {//&& getRuntime()<28
            pickupTimer -= lastRuntime-getRuntime();
            telemetry.addData("PICKUP TIMER", pickupTimer);
            robot.update();
            robot.getPayload().levelGripper();
            telemetry.update();
            lastRuntime = getRuntime();
        }
        robot.getChassis().stop();
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
        robot.getChassis().stop();
    }

    void goToPoseOvershoot(double x, double y, double angle, double speed, UniversalThreeWheelNavigator.Nav_Axis axis) throws InterruptedException {
        Pose2d startPose = robot.navigator.getMeasuredPose();
        double startVal = 0;
        switch (axis){
            case X:
                startVal = startPose.getX();
                break;
            case Y:
                startVal = startPose.getY();
                break;
            case ANGLE:
                startVal = startPose.getHeading();
                break;
        }
        while (robot.navigator.goTowardsPoseOvershoot(x, y * sideMultiplier, angle * sideMultiplier, speed, axis, startVal) && !isStopRequested()) {//&& getRuntime()<28
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
        robot.getChassis().stop();
    }

    void turnTo(double angle, double speed) throws InterruptedException {
        while (robot.navigator.turnTowards(sideMultiplier * angle, speed) && !isStopRequested()) {
            robot.update();
            robot.getPayload().levelGripper();
            telemetry.update();
        }
        robot.getChassis().stop();
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
        robot.getPayload().lift.goToPosition(polePose[0]-3);
        robot.getPayload().arm.goToPosition(polePose[1]);
        while (Math.abs(robot.getPayload().lift.getPosition()-polePose[0])>0.4
                || Math.abs(robot.getPayload().arm.getPosition()-polePose[1])>5 &&!isStopRequested()) {
            telemetry.addLine("Lifting 2");
            telemetry.update();
            robot.getPayload().levelGripper();}
        robot.getPayload().levelGripper();
        sleep(1000);
        robot.getPayload().toggleGripper(true);
        sleep(300);
    }

    void moveArmToPickup(int numCones){
        robot.getPayload().toggleGripper(true);
        robot.getPayload().lift.goToPosition(robot.getPayload().pickupPose[0]+(numCones*coneStackInterval)-1.4);
        robot.getPayload().arm.goToPosition(robot.getPayload().pickupPose[1]);
    }

    void pickUpCone(int numCones) throws InterruptedException {
        //goToPose(conePickupX,27,90,0.4);
        robot.getPayload().toggleGripper(true);
        robot.getPayload().lift.goToPosition(numCones*coneStackInterval);
        robot.getPayload().arm.goToPosition(robot.getPayload().pickupPose[1]);
        //lower
        while (Math.abs(robot.getPayload().lift.getPosition()-(numCones*coneStackInterval))>0.4
                && !isStopRequested()) {
            telemetry.addLine("Aligning");
            telemetry.update();
            robot.getPayload().levelGripper();}

        //pickup
        robot.getPayload().toggleGripper(false);
        sleep(400);
        //goToPose(conePickupX,24,90,0.4);

        //raise
        robot.getPayload().lift.goToPosition(robot.getPayload().pickupPose[0]+(numCones*coneStackInterval));
        while (Math.abs(robot.getPayload().lift.getPosition()-(robot.getPayload().pickupPose[0]+(numCones*coneStackInterval)-2))>0.4 && !isStopRequested()) {
            telemetry.addLine("Lifting 1");
            telemetry.update();
            robot.getPayload().levelGripper();}
        sleep(400);
    }


    void nextLights(){
        if(lightNum==0){
            robot.getPayload().lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if(lightNum==1){
            robot.getPayload().lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if(lightNum==2){
            robot.getPayload().lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
        else if(lightNum==3){
            robot.getPayload().lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        else if(lightNum==4){
            robot.getPayload().lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        else
        {
            robot.getPayload().lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        }
        lightNum++;
        if(lightNum>5){
            lightNum=0;
        }
    }
}