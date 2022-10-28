package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Juan Test Autonomous", group = "Juan")
@Config
public class TestAutonomous extends LinearOpMode
{
    Juan robot;
    public static double speed = -0.5;

    public static double driveSpeedThresh = 0.1;
    public static double driveSpeedTime = 0.2;

    public static double turnSpeedThresh = 0.1;
    public static double turnSpeedTime = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan(this,true,false,true);
        robot.init();

        waitForStart();
        robot.start();

        robot.getChassis().setHeadlessMode(true);

        while (!isStopRequested()){
            //driving path
        }

        robot.stop();
    }
}