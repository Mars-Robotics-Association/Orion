package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "JUAN FRONT - RELEASED", group = "JUAN")
@Config
public class JuanAutonomousFront_RELEASED extends LinearOpMode
{
    Juan_RELEASED robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan_RELEASED(this,true,true, false);
        robot.init();
        waitForStart();
        robot.start();
        robot.getChassis().setHeadlessMode(true);

        robot.getChassis().rawDrive(0, -5, 0);
        sleep(500);
        robot.getChassis().rawDrive(0, 0, 0);
    }
}
