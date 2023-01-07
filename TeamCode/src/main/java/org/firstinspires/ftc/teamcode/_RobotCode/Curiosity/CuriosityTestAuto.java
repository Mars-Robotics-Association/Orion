package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;

@Autonomous
@Config
public class CuriosityTestAuto extends LinearOpMode
{
    public static double[] p1 = {10,0,0};
    public static double[] p2 = {10,20,90};

    CuriosityBot robot;
    private ControllerInput controllerInput1;

    @Override
    public void runOpMode() throws InterruptedException {
        controllerInput1 = new ControllerInput(gamepad1, 1);
        robot = new CuriosityBot(this,controllerInput1,true,true,true);
        waitForStart();

        while(robot.navigator.goTowardsPose(p1[0],p1[1],p1[2],0.5))
        robot.navigator.goTowardsPose(p2[0],p2[1],p2[2],0.5);
    }
}