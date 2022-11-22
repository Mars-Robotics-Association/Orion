package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Navigation.UniversalThreeWheelNavigator;

@Config
class IngenuityNavigation extends UniversalThreeWheelNavigator
{
    ////Variables////
    public static double[] nav_encoderMultipliers = {1, -1, 1} ; //left right horizontal
    public static double nav_trackwidth = 7.3 ;
    public static double nav_centerWheelOffset = 1 ;  // Check for Ingy

    public static double nav_turnPID_P = 0.075 ;  // Default = 0.035
    public static double nav_turnPID_I = 0 ;  // Default = 0
    public static double nav_turnPID_D = -0.3 ;  // Default = -0.1

    public static double nav_movePID_D = 0.2 ;  // Default = 0.1
    public static double nav_movePID_I = 0 ;  // Default = 0
    public static double nav_movePID_P = 0.3 ;  // Default = 0.3

    public static double nav_stopSpeedThreshold = 0.15 ; //how slow the robot needs to be moving before it stops
    public static double nav_stopTimeThreshold = 0.2 ; //how long it needs to be below speed threshold

    public IngenuityNavigation(OpMode setOpMode, BaseRobot baseRobot, DistanceSensor setDistancePort, DistanceSensor setDistanceStarboard, ColorSensor setColorSensor) {

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
        return super.calculateMoveAngleSpeed(targetX, targetY, -speed);
    }

    /*  =================  AVAILABLE METHODS  =====================

    turnTowards(double targetAngle, double speed){return turnTowards(targetAngle, speed, stopSpeedThreshold, stopTimeThreshold)
    moveTowards(double targetX, double targetY, double speed)
    goTowardsPose(double targetX, double targetY, double targetAngle, double speed)
    */
}
