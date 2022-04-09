package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "*Ingenuity Ducky Depot*", group = "Ingenuity Competition")
public class IngenuityDuckyDepot extends LinearOpMode
{
    IngenuityControl robot;
    IngenuityDuckController duckController;

    public static double timeToDucksA = 2.1;
    public static double timeToDucksB = 2;
    public static double distanceToWallPark = 14;
    public static double moveToParkTime = 1.8;
    public static double spinTime = 18;
    public static double timeToWarehouse = 1.5;
    public static boolean redSide = true;

    double sideMultiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new IngenuityControl(this, true, true, false);
        robot.Init();
        duckController = robot.GetDuck();

        waitForStart();
        robot.Start();

        //depending on what side we are on, change the multiplier
        if(redSide) sideMultiplier = 1;
        else sideMultiplier = -1;

        // DUCK DELIVERY //

        //Start the spinner
        if(redSide) duckController.RedSide();
        else duckController.BlueSide();

        //Go to the duck spinner
        robot.rawDriveFor(timeToDucksA, -80, 0.5, 0);
        robot.rawDriveFor(timeToDucksB, -100, 0.5, 0);
        //wait
        Thread.sleep(robot.ToMillis(spinTime));

        duckController.Stop();

        // WAREHOUSE //

        //go to wall
        robot.rawDriveFor(timeToWarehouse, -15,0.5,0);
    }
}
