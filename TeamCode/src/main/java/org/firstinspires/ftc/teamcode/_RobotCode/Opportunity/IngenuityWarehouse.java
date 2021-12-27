package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.CuriosityRobot;
import org.firstinspires.ftc.teamcode._RobotCode.Ingenuity.IngenuityControl;

@Autonomous(name = "*Ingenuity Warehouse*", group = "Ingenuity")
@Config
public class IngenuityWarehouse extends LinearOpMode
{
    public static double time = 2;
    CuriosityRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CuriosityRobot(this, true, false, false);
        robot.Init();

        waitForStart();
        robot.Start();

        //go to wall
        double arriveAtWallTime = getRuntime();
        while (getRuntime() < arriveAtWallTime + time){
            robot.RawDrive(0,-1,0);
            if(!opModeIsActive()) return;
        }
        robot.RawDrive(0,0,0);
    }

}
