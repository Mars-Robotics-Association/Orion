package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.UniversalThreeWheelNavigator;

@Config
class JuanNavigation_RELEASED extends UniversalThreeWheelNavigator
{
    ////Variables////
    // Calibrated for Oppy 2023-02-05
    public static double[] nav_encoderMultipliers = {-1, -1, 1}; //left right horizontal
    public static double nav_trackwidth = 8.5;
    public static double nav_centerWheelOffset = -6.2;

    public static double nav_minSpeed = 0.18; //the min speed to move if not at the target location or rotation
    public static double nav_moveCoefficient = 0.08; //how aggressively to move
    public static double nav_moveSmoothCoefficient = 0.1; //how much to ramp movement into its final speed
    public static double nav_turnCoefficient = 0.02; //how aggressively to turn
    public static double nav_turnSmoothCoefficient = 0.1; //how much to ramp turning into its final speed
    public static double slowDistance = 0; //when to start slowing down

    public static double nav_stopDistance = .5; //inches away for robot to stop
    public static double nav_stopDegrees = 2; //degrees away for robot to stop
    public static double nav_stopTime = 0.1; //how long it needs to be below speed threshold

    public static double nav_moveSpeed = -1;
    public static double nav_turnSpeed = -1;

    ////DEPENDENCIES////
    //private final OpMode opMode;
    //private final MecanumChassis chassis;
    //public MecanumChassis getChassis(){return chassis;}

    public JuanNavigation_RELEASED(OpMode setOpMode, BaseRobot baseRobot) { //, DistanceSensor setDistancePort, DistanceSensor setDistanceStarboard, ColorSensor setColorSensor) {

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
    /*public JuanNavigation_RELEASED(OpMode setOpMode, BaseRobot baseRobot){
        opMode = setOpMode;
        chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("JUAN", 200, setOpMode), baseRobot);
    */
    }


    ////UTILITY////
    //private double getDistance(double x1, double y1, double x2, double y2){
    //    double xError = x1-x2;
    //    double yError = y1-y2;
    //    return Math.sqrt((xError*xError)+(yError*yError)); //return distance
    //}

    private double getMedian(double a, double b){
        return (a + b) / 2;
    }

    final double radToDegrees = 180 / Math.PI;
    final double DegreesToRad = 180 / Math.PI;
}
