package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.ams.AMSColorSensor;
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
        nav.NavigatorOn();
        nav.side = side;

        multiplier = -nav.GetSideMultiplier();
        robot.GetImu().OffsetGyro(-90);

//        //STARTS ALONG WALL BY DUCKS TODO start facing other way
//        nav.GoToWall(0.2);
//
//        //TURNS TO FACE AND SCAN TOWER TODO get rid of this
//        nav.DriveForTime(90*multiplier,0.5,0,0.5);
//        nav.TurnToAngle(-90*multiplier,0.2);
//        nav.TurnToAngle(-90*multiplier,0.1);
//        nav.DriveForTime(0,1,0,0.2);
//
//        nav.Wait(.5);
        //SCAN
        FreightFrenzyNavigation.DuckPos pos = nav.ScanBarcodeOpenCV();
        telemetry.update();

        //MOVE ARM TO POSITION
        if(pos == FreightFrenzyNavigation.DuckPos.FIRST) robot.TurretArm().GoToTier(CuriosityTurretArm.Tier.BOTTOM);
        else if(pos == FreightFrenzyNavigation.DuckPos.SECOND) robot.TurretArm().GoToTier(CuriosityTurretArm.Tier.MIDDLE);
        else robot.TurretArm().GoToTier(CuriosityTurretArm.Tier.TOP);

        //PLACES FREIGHT
        nav.DriveForTime(120*multiplier,0.7,0,1.3);
        robot.TurretArm().SetIntakeSpeed(-1);
        nav.Wait(2);
        nav.DriveForTime(-60*multiplier,0.5,0,0.5);

        //TURNS BACK TO WALL
        nav.TurnToAngle(0,0.5);
        robot.TurretArm().SetIntakeSpeed(0);
        robot.Arm().GoToPosition(0.02);

        //SPINS DUCKS
        nav.DriveAndSpinDucksLinear(1,0.5);

        //GOES TO PARK
        nav.WallFollowForTime(-1,0.5);
        nav.WallFollowToWhite(0.5,180);
        nav.DriveForTime(180,0.5,0,0.5);

        robot.Arm().GoToPosition(0);

        nav.StopNavigator();
        stop();


    }
}
