package org.firstinspires.ftc.teamcode.Samples.MechanicalControl;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MechanicalControl.MotorizedIntake;

public class IntakeSample
{
    private Servo servoR;
    private Servo servoL;

    public IntakeSample(Servo setServoR, Servo setServoL){
        servoR = setServoR;
        servoL = setServoL;
    }

    public void IntakeOn(){
        servoR.setPosition(1);
        servoL.setPosition(0);
    }
    public void IntakeOff(){
        servoR.setPosition(0.5);
        servoL.setPosition(0.5);
    }
}
