package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Navigation.Camera;

@Autonomous(name = "**JUAN AUTO RELEASED (AW)***", group="Oppy")
@Config

public class JuanAutonomous_RELEASED_AW extends LinearOpMode {
    private Juan_RELEASED robot;
    //Demobot robot;

    ////Variables////
    //Tweaking Vars
    public static double driveSpeed = 1;//used to change how fast robot drives
    public static double turnSpeed = -1;//used to change how fast robot turns
    private final double speedMultiplier = 1;

    private final double liftOverrideSpeed = 5;

    public static int payloadControllerNumber = 1;

    private JuanPayload_RELEASED payload;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan_RELEASED(this,true,true,true);

        robot.init();
        double coneSide = getConeSide(robot.camera);

        payload = robot.getPayload();
        JuanPayload_RELEASED.LiftController lift = payload.getLift();
        JuanPayload_RELEASED.GripperController gripper = payload.getGripper();

        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.update();


        //START
        waitForStart();
        robot.start();
//        resetRuntime();
//        robot.getChassis().resetGyro();


        //******************************************************************************************
        // CLOSE Gripper
        gripper.release();

        // Wait for Gripper to close
        sleep (250);

        // Raise Lift to HIGH
        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.HIGH);

        // Go to edge of middle of SQUARE 3
        goToPose(47, 0, 0, 1);

        // Set Lift height to Cruise
//        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.CRUISE);

        // Turn to High Pole
        goToPose(47, -1, -42, .7);

        // Wait for Lift
        while ( java.lang.Math.abs( lift.getLiftPosition() - lift.getLiftCurrentTargetPosition() ) > 50 ) {
            robot.update();
            telemetry.update();
        }


        // Move to High Pole
        goToPose( 54, -7,-42,.7);

        // Lower Lift to MEDIUM
        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.MEDIUM);

        // Wait for lift
        while ( java.lang.Math.abs( lift.getLiftPosition() - lift.getLiftCurrentTargetPosition() ) > 50 ) {
            robot.update();
            telemetry.update();
        }

        // Release Grip
        gripper.grab();

        // Back away to middle of SQUARE 3
        goToPose(47, -1, -42, .7);

        // Lower Gripper
        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.STACK_1);

        // Turn to Stack
        goToPose( 47,-1,90,.7 );

        // Drive to Stack
//        goToPose( 47, 20, 90, .7);
        goToPose( 47, 23.5, 90,.6);
        //sleep (250);

        // Grab Cone
        gripper.release();
        sleep (250);
        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.MEDIUM);

        // Back away to middle of SQUARE 3
        goToPose(47, -1, 90, 1);

        // Turn to Middle Pole
        goToPose(47, -1, 235, .7);

        // Go to Middle Pole
        goToPose(40,-7,235,.7);
        sleep (3000);
        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.LOW);

        // Wait for lift
        while ( java.lang.Math.abs( lift.getLiftPosition() - lift.getLiftCurrentTargetPosition() ) > 50 ) {
            robot.update();
            telemetry.update();
        }

        gripper.grab();

        // Back away to middle of SQUARE 3
        goToPose(47, -1, 180, .7);
        //goToPose(37, -3, 180, .7);

        // Go to center of SQUARE 2
        goToPose( 26,-3,180,1);

        // GREEN
        if(coneSide==1) {
            goToPose(23, -22, 180, 1);
        }
        // PURPLE
        else if(coneSide==2){
        // ORANGE
            // Do Nothing...
        }
        else{
            goToPose( 26, 20,180, 1);
            }
        //******************************************************************************************

        lift.goToPreset(JuanPayload_RELEASED.LiftHeight.BOTTOM);

        // Wait for lift
        while ( java.lang.Math.abs( lift.getLiftPosition() - lift.getLiftCurrentTargetPosition() ) > 50 ) {
            robot.update();
            telemetry.update();
        }


        telemetry.addLine("DONE");
        telemetry.update();
        robot.stop();
        stop();

    }

    //move this somewhere else if it goes in a different class
    double getConeSide(Camera c) throws InterruptedException {
        return 1;
        /*
        Bitmap img = c.getImage();
        Mat cropped = new Mat(c.convertBitmapToMat(img), new Rect(5 * img.getWidth() / 8, img.getHeight() / 3, img.getWidth() / 4, img.getHeight() / 3));
        Bitmap img2 = c.convertMatToBitMap(cropped);
        Mat in = c.convertBitmapToMat(c.shrinkBitmap(img2, 20, 20));
        //dash.sendImage(img);
        Mat greenMat = c.isolateColor(in, OpenCVColors.ConeGreenH, OpenCVColors.ConeGreenL);
        Mat purpleMat = c.isolateColor(in, OpenCVColors.ConePurpleH, OpenCVColors.ConePurpleL);
        Mat orangeMat = c.isolateColor(in, OpenCVColors.ConeOrangeH, OpenCVColors.ConeOrangeL);

        int greenCount = c.countPixels(c.convertMatToBitMap(greenMat));
        int purpleCount = c.countPixels(c.convertMatToBitMap(purpleMat));
        int orangeCount = c.countPixels(c.convertMatToBitMap(orangeMat));

        if(greenCount>purpleCount&&greenCount>orangeCount){
            //dash.sendImage(c.growBitmap(c.convertMatToBitMap(greenMat),200,200));
            return 1;}
        else if(purpleCount>greenCount&&purpleCount>orangeCount){
            //dash.sendImage(c.growBitmap(c.convertMatToBitMap(purpleMat),200,200));
            return 2;
        }
        else{
           // dash.sendImage(c.growBitmap(c.convertMatToBitMap(orangeMat),200,200));
            return 3;
        }
         */
        }

        //positive x is forward in inches, positive y is right in inches, use coordinates for x and y
        //right turn is positive angle in degrees

    void goToPose(double x, double y, double angle, double speed) throws InterruptedException {
        while(robot.navigator.goTowardsPose(x,y,angle,speed) && !isStopRequested()) {
            robot.update();
            telemetry.update();
        }

    }

 }

