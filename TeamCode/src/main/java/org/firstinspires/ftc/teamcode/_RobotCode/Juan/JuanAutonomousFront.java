package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "JUAN AUTO FRONT", group = "JUAN")
@Config
public class JuanAutonomousFront extends LinearOpMode
{
    Juan robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan(this,true,true, false);
        robot.init();
        waitForStart();
        robot.start();
        robot.getChassis().setHeadlessMode(true);

        robot.getChassis().rawDrive(0, -0.5, 0);
        sleep(1000);
        robot.getChassis().rawDrive(0, 0, 0);
    }
}
