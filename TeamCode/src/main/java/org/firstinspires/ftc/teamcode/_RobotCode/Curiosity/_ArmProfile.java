package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;

class _ArmProfile implements EncoderActuatorProfile
{
    DCMotorArray MOTORS;
    double MAX_ROTS = 290;
    double MIN_ROTS = 0;
    double GEAR_RATIO = 0.041666; // 384.5 / 23
    double ENCODER_RESOLUTION = 537.7; //384.5; //gobilda 13.7:1
    boolean REVERSE_ENCODER = true;
    boolean USE_ENCODER = true;

    public _ArmProfile(DcMotor motor1){
        MOTORS = new DCMotorArray(new DcMotor[]{motor1}, new double[]{-1},USE_ENCODER);
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
