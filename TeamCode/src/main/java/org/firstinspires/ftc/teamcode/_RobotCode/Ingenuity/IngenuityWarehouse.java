package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "*Ingenuity Warehouse*", group = "Ingenuity")
@Config
public class IngenuityWarehouse extends LinearOpMode
{
    public static double time = 2;
    IngenuityControl robot;
    private IngenuityDuckController duckController;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new IngenuityControl(this, true, true, false);
        robot.Init();
        duckController = new IngenuityDuckController(hardwareMap.servo.get("duckController"));

        waitForStart();
        robot.Start();

        //go to wall
        double arriveAtWallTime = getRuntime();
        while (getRuntime() < arriveAtWallTime + time){
            robot.RawDrive(0,-0.5,0);
            if(!opModeIsActive()) return;
        }
        robot.RawDrive(0,0,0);
    }

}
