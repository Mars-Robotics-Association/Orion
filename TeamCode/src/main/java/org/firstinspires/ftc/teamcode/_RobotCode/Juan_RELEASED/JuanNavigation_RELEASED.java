package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

@Config
class JuanNavigation_RELEASED
{
    ////DEPENDENCIES////
    private final OpMode opMode;
    private final MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}

    public JuanNavigation_RELEASED(OpMode setOpMode, BaseRobot baseRobot){
        opMode = setOpMode;
        chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("JUAN", 200, setOpMode), baseRobot);
    }

    public void update(){

    }

    ////UTILITY////
    private double getDistance(double x1, double y1, double x2, double y2){
        double xError = x1-x2;
        double yError = y1-y2;
        return Math.sqrt((xError*xError)+(yError*yError)); //return distance
    }

    private double getMedian(double a, double b){
        return (a + b) / 2;
    }

    final double radToDegrees = 180 / Math.PI;
    final double DegreesToRad = 180 / Math.PI;
}
