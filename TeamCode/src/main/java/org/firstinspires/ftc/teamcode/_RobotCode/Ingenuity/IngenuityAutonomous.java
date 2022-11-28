package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.Path;
import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.PathPoint;

@Autonomous(name = "Ingenuity Autonomous", group = "Ingenuity")
@Config
public class IngenuityAutonomous extends LinearOpMode
{
    IngenuityPowerPlayBot robot;
    public static double speed = -0.5;


    public static double driveSpeedThresh = 0.1;
    public static double driveSpeedTime = 0.2;

    public static double turnSpeedThresh = 0.1;
    public static double turnSpeedTime = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new IngenuityPowerPlayBot(this,true,true,true);
        robot.init();

        waitForStart();
        robot.start();

        robot.getChassis().setHeadlessMode(true);



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
