package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.CuriosityRobot;
import org.firstinspires.ftc.teamcode._RobotCode.Ingenuity.IngenuityControl;

@Config
@Autonomous(name = "*Oppy Blue Duck*", group = "Oppy")
@Disabled
public class IngenuityBlueDucks extends LinearOpMode
{
    CuriosityRobot robot;
    Servo duckController;

    public static double timeToDucks = 2;
    public static double distanceToWallPark = 14;
    public static double moveToParkTime = 1.8;
    public static double spinTime = 20;
    public static boolean redSide = false;
    double sideMultiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CuriosityRobot(this, true, false, false);
        robot.Init();
        duckController = hardwareMap.servo.get("duckyServo");

        waitForStart();
        robot.Start();

        //depending on what side we are on, change the multiplier
        if(redSide) sideMultiplier = 1;
        else sideMultiplier = -1;

        //Start the spinner
        if(redSide) duckController.setPosition(0);
        else duckController.setPosition(1);

        //Go to the duck spinner
        double startTime = getRuntime();
        while (getRuntime() < startTime + timeToDucks){
            robot.RawDrive(0, 0.5, 0.02*sideMultiplier); //add slight turn into the wall
            if(!opModeIsActive()) return;
        }

        //wait
        double arriveAtWallTime = getRuntime();
        while (getRuntime() < arriveAtWallTime + spinTime){
            robot.RawDrive(0,0.1,sideMultiplier*0.04);
            if(!opModeIsActive()) return;
        }

        duckController.setPosition(0.5);

        //move to park
        double parkStartTime = getRuntime();
        while (getRuntime() < parkStartTime + moveToParkTime){
            robot.RawDrive(90+(sideMultiplier*10),sideMultiplier*0.5,0); //TODO: check this
            if(!opModeIsActive()) return;
        }
        double intialParkTime = getRuntime();
        while (getRuntime() < arriveAtWallTime + 1){
            robot.RawDrive(0, 0.5, 0);
            if(!opModeIsActive()) return;
        }
        robot.RawDrive(0,0,0);

    }
}
