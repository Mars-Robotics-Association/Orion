package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Input.ControllerInput;
import org.firstinspires.ftc.teamcode.Orion.OrionNavigator;
import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.CuriosityUltimateGoalControl;

//The class for controlling the robot in teleop. Includes basic drive movement, shooter operations,
//and advanced autonomous functions.

//REQUIRED TO RUN: Phones | REV Hub | Demobot Chassis | Shooter | Odometry Unit
//REQUIRED TO FUNCTION: Controllers

@Config
@Autonomous(name = "*Testing OpMode*")
public class NavigationTesting extends LinearOpMode {
    private CuriosityUltimateGoalControl control;
    private OrionNavigator orion;
    private FtcDashboard dashboard;

    public static double tfDistCoef = 6666;
    public static double tfXCoef = 0.001;

    public static double robotX = 0;
    public static double robotY = 4;
    public static double robotH = 180;

    public static double waitTime = 500.0;

    public static double tfUpperLimit = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        control = new CuriosityUltimateGoalControl(this,true,true,true);
        control.Init();
        orion = control.GetOrion();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();
        orion.SetPose(robotX, robotY, Math.toRadians(robotH));//robot starts on red right line

        //put the starpath in the right place
        control.StarpathToStorage();

        //Move to where it can see discs
        orion.MoveLine(18, 4, 0);

        //wait a bit for it to see discs
        sleep(500);//wait for tensorflow to detect discs
        int numberOfDiscs = orion.GetNumberOfDiscs(tfUpperLimit);//figure out where to go\

        telemetry.addData("Number of discs", numberOfDiscs);
        telemetry.update();
        sleep(10000);

    }
}