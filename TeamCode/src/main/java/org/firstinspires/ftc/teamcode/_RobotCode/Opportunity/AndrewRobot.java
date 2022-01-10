package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

public class AndrewRobot extends MecanumChassis {
    AndrewArmNew andrewArm;


    public AndrewRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new _ChassisProfile(), new HermesLog("Opportunity", 500, setOpMode), useChassis, usePayload, useNavigator);
        if(usePayload){
            andrewArm = new AndrewArmNew(opMode,new _ArmProfile(opMode.hardwareMap.dcMotor.get("armPosition")),opMode.hardwareMap.dcMotor.get("clawMotor"),false,opMode.hardwareMap.dcMotor.get("armPosition"));

        }
    }
    public AndrewArmNew getAndrewArm(){
        return andrewArm;
    }
    public void start(){
        super.StartCoreRobotModules();
    }





}
