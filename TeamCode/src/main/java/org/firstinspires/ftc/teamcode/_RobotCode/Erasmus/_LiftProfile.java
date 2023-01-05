package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;

class _LiftProfile implements EncoderActuatorProfile
{
    DCMotorArray MOTORS;
    double MAX_ROTS = 1000;
    double MIN_ROTS = 0;
    double GEAR_RATIO = 1;
    double ENCODER_RESOLUTION = 537.7; //gobilda 19.2:1
    boolean REVERSE_ENCODER = false;
    boolean USE_ENCODER = true;

    public _LiftProfile(DcMotor rightMotor, DcMotor leftMotor, double startPos){
        MOTORS = new DCMotorArray( new DcMotor[] {rightMotor, leftMotor}, new double[]{0.8,0.8},USE_ENCODER);
        MAX_ROTS -= startPos;
        MIN_ROTS -= startPos;
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
