package org.firstinspires.ftc.teamcode.Navigation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;

public class SampleNavigator extends UniversalThreeWheelNavigator
{

    ////Variables////
    public static double[] nav_encoderMultipliers = {1, 1, -1}; //left right horizontal
    public static double nav_trackwidth = 10.7;
    public static double nav_centerWheelOffset = -3;

    public static double nav_turnPID_P = 0.035;
    public static double nav_turnPID_I = 0;
    public static double nav_turnPID_D = -0.1;

    public static double nav_movePID_D = 0.1;
    public static double nav_movePID_I = 0;
    public static double nav_movePID_P = 0.3;

    public static double nav_stopSpeedThreshold = 0.1; //how slow the robot needs to be moving before it stops
    public static double nav_stopTimeThreshold = 0.2; //how long it needs to be below speed threshold

    public SampleNavigator(OpMode setOpMode, BaseRobot baseRobot, DistanceSensor setDistancePort, DistanceSensor setDistanceStarboard, ColorSensor setColorSensor) {

        ////CONFIGURABLE////
        encoderMultipliers = nav_encoderMultipliers;
        trackwidth = nav_trackwidth;
        centerWheelOffset = nav_centerWheelOffset;

        turnPID_P = nav_turnPID_P;
        turnPID_I = nav_turnPID_I;
        turnPID_D = nav_turnPID_D;

        movePID_D = nav_movePID_D;
        movePID_I = nav_movePID_I;
        movePID_P = nav_movePID_P;

        stopSpeedThreshold = nav_stopSpeedThreshold; //how slow the robot needs to be moving before it stops
        stopTimeThreshold = nav_stopTimeThreshold; //how long it needs to be below speed threshold

        InitializeNavigator(setOpMode, baseRobot, setDistancePort, setDistanceStarboard, setColorSensor);
    }

    //make these speeds negative if going wrong direction
    @Override
    protected double calculateTurnSpeed(double targetAngle, double speed){
        return super.calculateTurnSpeed(targetAngle,-speed);
    }

    @Override
    protected double[] calculateMoveAngleSpeed(double targetX, double targetY, double speed){
        return super.calculateMoveAngleSpeed(targetX, targetY, speed);
    }
}