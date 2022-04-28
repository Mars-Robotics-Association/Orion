package org.firstinspires.ftc.teamcode._RobotCode.Archived.Curiosity;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;

class _TurretProfile implements EncoderActuatorProfile
{
    DcMotor MOTOR;
    double MAX_ROTS = 0.5;
    double MIN_ROTS = -0.5;
    double GEAR_RATIO = 9;
    double ENCODER_RESOLUTION = 537.7; //gobilda 19.2:1
    boolean REVERSE_ENCODER = false;
    boolean USE_ENCODER = true;

    public _TurretProfile(DcMotor motor){
        MOTOR = motor;
    }

    @Override
    public MotorArray motors() {return new MotorArray(new DcMotor[]{MOTOR}, new double[]{1}, USE_ENCODER);}

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
