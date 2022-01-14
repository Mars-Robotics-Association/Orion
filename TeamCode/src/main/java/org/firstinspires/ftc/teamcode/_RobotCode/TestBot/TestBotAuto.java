package org.firstinspires.ftc.teamcode._RobotCode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test Bot Auto", group = "test bot")
@Disabled
public class TestBotAuto extends LinearOpMode
{
    TestBot robot;

    double segmentTime = 0.5;
    double waitTime = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new TestBot(this,true,false,false);
        robot.Init();

        waitForStart();
        robot.Start();

        GoForTime(90,0.5,0, 1); //strafo away from wall
        StopAndWait(waitTime);
        GoForTime(-90,0.5,0, 1.1);  //strafo into wall
        StopAndWait(waitTime);
        GoForTime(0,0.5,0.03, 1.8); //drive straight 36in
        StopAndWait(waitTime);
        GoForTime(90,0.5,0, .5);  //strafo into middle
        StopAndWait(waitTime);


    }

    public void GoForTime(double angle, double speed, double turnOffset, double time){
        double startTime = getRuntime();
        while (getRuntime() < startTime + time){
            robot.RawDrive(angle,speed,turnOffset);
            if(!opModeIsActive()) return;
        }
    }
    public void StopAndWait(double time){
        double startTime = getRuntime();
        while (getRuntime() < startTime + time){
            robot.RawDrive(0,0,0);
            if(!opModeIsActive()) return;
        }
    }
}
