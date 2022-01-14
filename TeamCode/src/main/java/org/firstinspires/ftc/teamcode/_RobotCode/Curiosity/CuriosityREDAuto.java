package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Orion.NavModules.Camera;
import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavigation;
import org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.RoadrunnerModule;

@Autonomous(name = "RED Curiosity Auto", group = "Curiosity")
@Config
public class CuriosityREDAuto extends LinearOpMode
{
    CuriosityRobot robot;
    Camera cam;
    FreightFrenzyNavigation nav;
    public static double targetHeading = -90;
    public static double targetHeading2 = -200;
    private double multiplier = 1;
    protected FreightFrenzyNavigation.AllianceSide side = FreightFrenzyNavigation.AllianceSide.RED;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CuriosityRobot(this, true, true, true);
        nav = robot.navigation;
        robot.Init();

        waitForStart();
        robot.Start();
        nav.side = side;

        multiplier = -nav.GetSideMultiplier();

        //STARTS ALONG WALL BY DUCKS
        nav.GoToWall(0.2);

        //TURNS TO FACE AND SCAN TOWER
        nav.DriveForTime(90*multiplier,0.5,0,0.5);
        nav.TurnToAngle(-90*multiplier,0.2);
        nav.TurnToAngle(-90*multiplier,0.1);
        nav.DriveForTime(0,1,0,0.2);
//        FreightFrenzyNavigation.DuckPos pos = nav.ScanBarcodeOpenCV(); //TODO: get working

        //PLACES FREIGHT
        robot.TurretArm().GoToTier(CuriosityTurretArm.Tier.MIDDLE);
        nav.DriveForTime(120*multiplier,0.7,0,1.4);
        robot.TurretArm().SetIntakeSpeed(-1);
        nav.Wait(1);
        nav.DriveForTime(-60*multiplier,0.5,0,0.5);

        //TURNS BACK TO WALL
        nav.TurnToAngle(0,0.5);
        robot.TurretArm().SetIntakeSpeed(0);
        robot.Arm().GoToPosition(0);

        //SPINS DUCKS
        nav.DriveAndSpinDucksLinear(1,0.5);

        //GOES TO PARK
        nav.WallFollowToWhite(0.5,180);
        nav.DriveForTime(180,0.5,0,0.5);

        stop();


    }
}
