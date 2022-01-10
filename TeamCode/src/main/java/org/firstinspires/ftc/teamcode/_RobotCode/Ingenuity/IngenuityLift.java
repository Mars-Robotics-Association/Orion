package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;

public class IngenuityLift extends EncoderActuator
{
    public IngenuityLift(OpMode setOpMode, DcMotor liftMotor) {
        super(setOpMode, new _IngenuityLiftProfile(liftMotor));
    }
}
