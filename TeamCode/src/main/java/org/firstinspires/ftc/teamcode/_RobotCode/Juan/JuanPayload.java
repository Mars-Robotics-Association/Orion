package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;

class JuanPayload
{
    OpMode opMode;

    //initializer
    public JuanPayload(OpMode setOpMode, EncoderActuator lift) {
        opMode = setOpMode;
    }

    ////BASIC FUNCTIONS////

    //print telemetry
    public void printTelemetry(){
        opMode.telemetry.addLine("----PAYLOAD----");
        opMode.telemetry.addData("absolutely ", "nothing");
    }
}
