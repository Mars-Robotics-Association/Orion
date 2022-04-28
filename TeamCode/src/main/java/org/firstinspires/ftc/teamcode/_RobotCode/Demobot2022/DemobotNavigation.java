package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

class DemobotNavigation
{
    private MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}

    public DemobotNavigation(OpMode setOpMode, BaseRobot baseRobot){
        chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("Demobot", 200, setOpMode), baseRobot);
    }
}
