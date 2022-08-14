package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.Path;
import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.PathPoint;

@Autonomous(name = "Demobot Test Autonomous", group = "Demobot")
public class TestAutonomous extends LinearOpMode
{
    Demobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Demobot(this,true,false,true);
        robot.init();

        waitForStart();
        robot.start();

        //do something?

        robot.stop();
    }
}
