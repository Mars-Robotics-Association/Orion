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
    public static double speed = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new IngenuityPowerPlayBot(this,true,true,true);
        robot.init();

        waitForStart();
        robot.start();

        robot.getChassis().setHeadlessMode(true);

        autoDrive();

        robot.stop();
    }

    private void goToPose(double x, double y, double angle) {
        while(! robot.getNavigator().goTowardsPose(x, y, angle,0.3) && !isStopRequested()) {
            robot.update();
            telemetry.addData("Going to to ", "("+x+", "+y+", "+angle+")");
            telemetry.update();
        }
    }

    public void autoDrive() {
        // Move in rectangle, clockwise around the post
        // Move forward to 16x, 0y
        goToPose(20,0,0);

        // Strafe right to 16v, 16y
        goToPose(20,20,0);

        // Move backward to 0x, 16y
        goToPose(0,20,0);

        // strafe left to 0x, 0y
        goToPose(0,0,0);
    }
}
