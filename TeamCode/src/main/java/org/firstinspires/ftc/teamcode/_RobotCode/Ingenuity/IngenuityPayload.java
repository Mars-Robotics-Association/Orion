package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;

class IngenuityPayload
{
    OpMode opMode;
    MotorArray intake;
    MotorArray path;
    MotorArray loader;
    EncoderActuator turret;
    MotorArray shooter;

    //internal
    boolean intakeOn = false;
    boolean pathOn = false;
    boolean shooterOn = false;

    //initializer
    public IngenuityPayload(OpMode setOpMode){

    }


    ////BASIC FUNCTIONS////
    //print telemetry
    public void printTelemetry(){


    }

    ////SENSORS////

}
