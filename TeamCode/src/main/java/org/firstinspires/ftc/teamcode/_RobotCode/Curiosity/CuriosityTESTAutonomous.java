package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Orion.NavModules.Camera;
import org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.RoadrunnerModule;

@Autonomous(name = "TEST Curiosity Auto", group = "Curiosity")
@Config
public class CuriosityTESTAutonomous extends LinearOpMode
{
    CuriosityRobot robot;
    RoadrunnerModule nav;
    Camera cam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CuriosityRobot(this, true, true, false);
        cam = new Camera(this,"Webcam 1");
        robot.Init();
        //nav = robot.Roadrunner();

        waitForStart();
        robot.Start();


        //detect duck


        //test some things
        nav.MoveLine(10,10,0);
        nav.Turn(90);

    }
}
