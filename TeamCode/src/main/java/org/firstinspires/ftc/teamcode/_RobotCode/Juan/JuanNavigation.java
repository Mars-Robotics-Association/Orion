package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

@Config
class JuanNavigation
{
    ////DEPENDENCIES////
    private OpMode opMode;
    private MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}

    double lastTimeAboveStopThreshold = 0;

    public JuanNavigation(OpMode setOpMode, BaseRobot baseRobot){
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
}
