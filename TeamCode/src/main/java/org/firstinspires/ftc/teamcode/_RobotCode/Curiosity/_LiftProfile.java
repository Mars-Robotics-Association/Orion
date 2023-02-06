package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;

class _LiftProfile implements EncoderActuatorProfile
{
    DCMotorArray MOTORS;
    double MAX_ROTS = 12;
    double MIN_ROTS = -4;
    double GEAR_RATIO = 0.424; //TODO: find this number
    double ENCODER_RESOLUTION = 537.7; //gobilda 20:1
    boolean REVERSE_ENCODER = true;
    boolean USE_ENCODER = true;

    public _LiftProfile(DcMotor motor1, DcMotor motor2){
        MOTORS = new DCMotorArray(new DcMotor[]{motor1,motor2}, new double[]{1,1},USE_ENCODER);
    }

    @Override
    public DCMotorArray motors() {return MOTORS;}

    @Override
    public double maxRots() {return MAX_ROTS;}

    @Override
    public double minRots() {return MIN_ROTS;}

    @Override
    public double gearRatio() {return GEAR_RATIO;}

    @Override
    public double encoderResolution() {return ENCODER_RESOLUTION;}

    @Override
    public boolean reverseEncoder() {return REVERSE_ENCODER;}

    @Override
    public boolean useEncoder() {return USE_ENCODER;}
}
