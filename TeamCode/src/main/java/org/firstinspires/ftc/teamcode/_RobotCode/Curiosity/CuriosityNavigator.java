package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Navigation.UniversalThreeWheelNavigator;

@Config
public class CuriosityNavigator extends UniversalThreeWheelNavigator
{
    ////Variables////
    public static double[] nav_encoderMultipliers = {1, 1, -1}; //left right horizontal
    public static double nav_trackwidth = 9.2;
    public static double nav_centerWheelOffset = -4;

    public static double nav_minSpeed = 0.2; //the min speed to move if not at the target location or rotation
    public static double nav_moveCoefficient = 0.06; //how aggressively to move
    public static double nav_moveSmoothCoefficient = 0.1; //how much to ramp movement into its final speed
    public static double nav_turnCoefficient = 0.01; //how aggressively to turn
    public static double nav_turnSmoothCoefficient = 0.2; //how much to ramp turning into its final speed


    public static double nav_stopDistance = .5; //inches away for robot to stop
    public static double nav_stopDegrees = 3; //degrees away for robot to stop
    public static double nav_stopTime = 0.1; //how long it needs to be below speed threshold

    public static double nav_moveSpeed = -1;
    public static double nav_turnSpeed = -1;

    public CuriosityNavigator(OpMode setOpMode, BaseRobot baseRobot, DistanceSensor setDistancePort, DistanceSensor setDistanceStarboard, ColorSensor setColorSensor) {

        encoderMultipliers = nav_encoderMultipliers;
        trackwidth = nav_trackwidth;
        centerWheelOffset = nav_centerWheelOffset;
        minSpeed = nav_minSpeed;
        moveCoefficient = nav_moveCoefficient;
        moveSmoothCoefficient = nav_moveSmoothCoefficient;
        turnCoefficient = nav_turnCoefficient;
        turnSmoothCoefficient = nav_turnSmoothCoefficient;
        stopDistance = nav_stopDistance;
        stopDegrees = nav_stopDegrees;
        stopTime = nav_stopTime;
        moveSpeedMultiplier = nav_moveSpeed;
        turnSpeedMultiplier = nav_turnSpeed;

        InitializeNavigator(setOpMode, baseRobot);
    }
}
