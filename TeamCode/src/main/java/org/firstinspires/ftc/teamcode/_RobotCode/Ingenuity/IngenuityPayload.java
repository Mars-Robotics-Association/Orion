package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;

class IngenuityPayload {
    OpMode opMode;
    EncoderActuator arm;

    //internal
    boolean intakeOn = false;
    boolean pathOn = false;
    boolean shooterOn = false;

    private final DcMotor motor;

    //initializer
    public IngenuityPayload(OpMode setOpMode, DcMotor armMotor) {
        opMode = setOpMode;
        motor = armMotor;
        arm = new EncoderActuator(opMode, new _ArmProfile(armMotor, 0));
    }

    public void ResetArmHomePosition(double pos) {
        arm = new EncoderActuator(opMode, new _ArmProfile(motor, pos));
    }


    ////BASIC FUNCTIONS////

    public EncoderActuator getArm() {
        return arm;
    }

}
