package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "*SIMPLE CURIOSITY AUTO*", group = "Curiosity")
public class DirtSimpleCuriosityAutonomous extends LinearOpMode
{
    CuriosityRobot robot;

    public static double distanceFromWallToStopCM = 8;
    public static double spinTime = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CuriosityRobot(this, true, true, false);

        waitForStart();

        robot.DuckSpinner().Forwards();

        while (robot.GetDistToWallCM() > distanceFromWallToStopCM){ //while not in range of the wall to spin ducks, move towards it
            robot.RawDrive(0, 0.5, -0.05); //add slight turn into the wall
        }
        robot.RawDrive(0,0,0);

        double arriveAtWallTime = getRuntime();
        while (getRuntime() < arriveAtWallTime + spinTime){
            //wait
        }

        //move to park
        //robot.RawDrive(90,0.5,0);

    }
}
