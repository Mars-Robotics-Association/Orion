package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Core.OrionObject;

class DemobotNavigation extends OrionObject
{
    private MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}

    public DemobotNavigation(OpMode setOpMode){
        SetOpModeNameVersion(setOpMode,"DemobotNavigation", 1.0);
    }

    public void SetUpNavigator(BaseRobot baseRobot){
        chassis = new MecanumChassis(opMode, new _ChassisProfile(), new HermesLog("Demobot", 200, opMode), baseRobot);

    }
}
