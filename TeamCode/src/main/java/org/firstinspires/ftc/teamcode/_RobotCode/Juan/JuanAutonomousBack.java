package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Camera;
import org.opencv.core.Mat;

@Autonomous(name = "JUAN BACK", group = "JUAN")
@Config
public class JuanAutonomousBack extends LinearOpMode
{
    Juan robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan(this,true,true, false);
        robot.init();

        waitForStart();
        robot.start();
        robot.getChassis().setHeadlessMode(true);

        robot.getChassis().rawDrive(0, 5, 0);
        sleep(500);
        robot.getChassis().rawDrive(0, 0, 0);
    }
}
