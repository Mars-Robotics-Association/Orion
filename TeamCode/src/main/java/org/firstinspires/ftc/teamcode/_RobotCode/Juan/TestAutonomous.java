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
    public static double x1 = 12;
    public static double y1 = 12;
    public static double x2 = 0;
    public static double y2 = -12;
    public static double x3 = 0;
    public static double y3 = 0;
    public static double a1 = 90;
    public static double a2 = -180;
    public static double a3 = 0;

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
            goTo(x1,y1,a1);
            turn(a1);
            move(x2,y2);
            turn(a2);
            move(x3,y3);
            turn(a3);

        }

        robot.stop();
    }

    private void move(double x, double y){
        while (!robot.getNavigator().moveTowards(x,y,speed,driveSpeedThresh,driveSpeedTime)&&!isStopRequested()){
            robot.update();
            telemetry.addData("Moving to ", "("+x+", "+y+")");
            telemetry.update();
        }
    }
    private void turn(double a){
        while (!robot.getNavigator().turnTowards(a,speed,turnSpeedThresh,turnSpeedTime)&&!isStopRequested()){
            robot.update();
            telemetry.addData("Turning to ", a);
            telemetry.update();
        }
    }
    private void goTo(double x, double y, double a){
        while (!robot.getNavigator().goTowardsPose(x,y,a,speed,turnSpeedThresh,turnSpeedTime)&&!isStopRequested()){
            robot.update();
            telemetry.addData("Going to to ", "("+x+", "+y+", "+a+")");
            telemetry.update();
        }
    }
}
