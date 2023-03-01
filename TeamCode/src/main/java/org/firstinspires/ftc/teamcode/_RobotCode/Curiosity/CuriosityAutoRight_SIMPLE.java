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
import org.firstinspires.ftc.teamcode.Navigation.UniversalThreeWheelNavigator.Nav_Axis;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

@Autonomous(name = "*SIMPLE CURIOSITY AUTO RIGHT*", group="Curiosity")
@Config
public class CuriosityAutoRight_SIMPLE extends LinearOpMode {

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

    double conePickupX = 45;
    double conePickupY = 29;

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
        isLeft = false;
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
        goToPoseOvershoot(55, 0, 0, 1,Nav_Axis.X);
        goToPose(48, 0, 0, 1);
        goToPose(47.5, -3.5, -45, speed);//goes to place
        turnTo(-45,0.6);
        //places cone
        deployCone(CuriosityPayload.Pole.HIGH);
        nextLights();

        //1=stack for left, 1=far for right
        //spot 1(green)
        if ((coneSide == 1 && isLeft) || (coneSide == 3 && !isLeft)) {
            moveArmToPickup(2);
            goToPoseNoLR(40, -20, 0, 1);
        }
        //spot 2(purple)
        else if (coneSide == 2) {
            moveArmToPickup(2);
            goToPoseNoLR(40, 0, 0, 1);
        }
        //spot 3(orange)
        else {
            moveArmToPickup(2);
            goToPoseNoLR(40, 24, 0, 1);
        }
        telemetry.addLine("DONE");
        telemetry.update();
        robot.stop();
        robot.getChassis().stop();
        stop();
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

    void goToPoseOvershoot(double x, double y, double angle, double speed, Nav_Axis axis) throws InterruptedException {
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
        robot.getPayload().lift.goToPosition(polePose[0]);
        robot.getPayload().arm.goToPosition(polePose[1]);
        while (Math.abs(robot.getPayload().lift.getPosition()-polePose[0])>0.4
                || Math.abs(robot.getPayload().arm.getPosition()-polePose[1])>5 &&!isStopRequested()) {
            telemetry.addLine("Lifting 2");
            telemetry.update();
            robot.getPayload().levelGripper();}

        robot.getPayload().toggleGripper(true);
        sleep(300);
    }

    void moveArmToPickup(int numCones){
        robot.getPayload().toggleGripper(true);
        robot.getPayload().lift.goToPosition(robot.getPayload().pickupPose[0]+(numCones*coneStackInterval)-1.4);
        robot.getPayload().arm.goToPosition(robot.getPayload().pickupPose[1]);
    }

    void pickUpCone(int numCones) throws InterruptedException {
        goToPose(conePickupX,27,90,0.4);
        robot.getPayload().toggleGripper(true);
        robot.getPayload().lift.goToPosition(numCones*coneStackInterval-0);
        robot.getPayload().arm.goToPosition(robot.getPayload().pickupPose[1]);
        //lower
        while (Math.abs(robot.getPayload().lift.getPosition()-(numCones*coneStackInterval))>0.4
                || Math.abs(robot.getPayload().arm.getPosition()-robot.getPayload().pickupPose[1])>5 && !isStopRequested()) {
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