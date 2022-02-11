package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavigation;

@Autonomous(name = "TEST Curiosity Auto", group = "Curiosity")
@Config
//@Disabled
public class CuriosityTESTAutonomous extends LinearOpMode
{
    CuriosityRobot robot;
    FreightFrenzyNavigation nav;
    public static double targetHeading = -90;
    public static double targetHeading2 = -200;
    private double multiplier = 1;
    protected FreightFrenzyNavigation.AllianceSide side = FreightFrenzyNavigation.AllianceSide.BLUE;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CuriosityRobot(this, true, true, true);
        nav = robot.navigation;
        robot.Init();

        waitForStart();
        robot.Start();
        nav.side = side;

        multiplier = -nav.GetSideMultiplier();



        nav.TurnToHubLinear();



    }
}
