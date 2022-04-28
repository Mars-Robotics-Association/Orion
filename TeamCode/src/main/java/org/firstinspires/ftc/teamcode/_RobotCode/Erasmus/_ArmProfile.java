package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;

class  _ArmProfile implements EncoderActuatorProfile
{
    MotorArray MOTORS;
    double MAX_ROTS = 0.29;
    double MIN_ROTS = 0;
    double GEAR_RATIO = 24;
    double ENCODER_RESOLUTION = 537.7; //gobilda 19.2:1
    boolean REVERSE_ENCODER = true;
    boolean USE_ENCODER = true;

    public _ArmProfile(DcMotor motor){
        MOTORS = new MotorArray(new DcMotor[]{motor}, new double[]{1},USE_ENCODER);
    }

    @Override
    public MotorArray motors() {return MOTORS;}

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
