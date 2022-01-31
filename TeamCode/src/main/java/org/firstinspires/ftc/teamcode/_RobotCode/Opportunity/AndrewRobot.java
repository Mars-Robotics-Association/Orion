package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

public class AndrewRobot extends MecanumChassis {
    DcMotor duckyMotor;
    DistanceSensor sideDist;
    DcMotor armPos;
    DcMotor turntable;
    DcMotor gripper;

    public void init(){
        duckyMotor = super.opMode.hardwareMap.dcMotor.get("duckyMotor");
        sideDist = super.opMode.hardwareMap.get(DistanceSensor.class, "distSide");
        armPos = super.opMode.hardwareMap.dcMotor.get("armPosition");
        turntable = super.opMode.hardwareMap.dcMotor.get("turntable");
        gripper = super.opMode.hardwareMap.dcMotor.get("clawMotor");

    }

    public AndrewRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new _ChassisProfile(), new HermesLog("Opportunity", 500, setOpMode), useChassis, usePayload, useNavigator);
    }

    public void wait(double time){
        double startTime = opMode.getRuntime();
        while (opMode.getRuntime()<startTime+time){ }

    }


    public void start(){
        super.StartCoreRobotModules();
    }





}
