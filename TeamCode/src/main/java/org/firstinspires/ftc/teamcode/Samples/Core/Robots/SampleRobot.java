package org.firstinspires.ftc.teamcode.Samples.Core.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.BaseRobots.MecanumBaseControl;
import org.firstinspires.ftc.teamcode.Samples.MechanicalControl.IntakeSample;

public class SampleRobot extends MecanumBaseControl
{
    public IntakeSample payload;

    public SampleRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, useChassis, usePayload, useNavigator);
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===
        if(USE_PAYLOAD) {
            Servo rightIntakeServo = currentOpMode.hardwareMap.servo.get("intake right");
            Servo leftIntakeServo = currentOpMode.hardwareMap.servo.get("intake left");

            payload = new IntakeSample(rightIntakeServo, leftIntakeServo);
        }

        //TODO ===INIT CORE ROBOT===
        super.InitCoreRobotModules();

        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){
        super.StartCoreRobotModules();
    }

    public void IntakeOn(){payload.IntakeOn();}
    public void IntakeOff(){payload.IntakeOff();}
}
