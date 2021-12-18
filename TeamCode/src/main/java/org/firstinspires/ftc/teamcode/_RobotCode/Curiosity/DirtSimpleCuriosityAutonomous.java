package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "*SIMPLE CURIOSITY AUTO*", group = "Curiosity")
public class DirtSimpleCuriosityAutonomous extends LinearOpMode
{
    CuriosityRobot robot;

    public static double distanceFromWallToStopCM = 36;
    public static double moveToParkTime = 2;
    public static double spinTime = 20;
    public static boolean redSide = true;
    double sideMultiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CuriosityRobot(this, true, true, false);
        robot.Init();

        waitForStart();
        robot.Start();

        //depending on what side we are on, change the multiplier
        if(redSide) sideMultiplier = 1;
        else sideMultiplier = -1;

        //Start the spinner
        if(redSide) robot.DuckSpinner().Reverse();
        else robot.DuckSpinner().Forwards();

        //Go to the duck spinner
        while (robot.GetDistToWallCM() > distanceFromWallToStopCM){ //while not in range of the wall to spin ducks, move towards it
            robot.RawDrive(0, 0.5, 0.02*sideMultiplier); //add slight turn into the wall
            if(!opModeIsActive()) return;
        }

        //wait
        double arriveAtWallTime = getRuntime();
        while (getRuntime() < arriveAtWallTime + spinTime){
            robot.RawDrive(0,0.1,sideMultiplier*0.04);
            if(!opModeIsActive()) return;
        }

        robot.DuckSpinner().Stop();

        //move to park
        double parkStartTime = getRuntime();
        while (getRuntime() < parkStartTime + moveToParkTime){
            robot.RawDrive(90,sideMultiplier*0.5,0);
            if(!opModeIsActive()) return;
        }
        robot.RawDrive(0,0,0);

    }
}