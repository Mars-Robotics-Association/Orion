package org.firstinspires.ftc.teamcode._RobotCode.Archived.Curiosity;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;

class _ChassisProfile implements ChassisProfile
{
    public _ChassisProfile(){
/*        flipIMU = false;
        motorNames = new String[]{"FR", "FL", "RR", "RL"};
        headingPID = new double[]{0, 0, 0};
        speedPID = new double[]{0, 0, 0};
        directionPID = new double[]{0, 0, 0};
        useEncoders = false;*/
    }

    @Override
    public boolean flipIMU() { return false; }
    @Override
    public String[] motorNames() { return new String[]{"FR", "FL", "RR", "RL"}; }
    @Override
    public double[] headingPID() { return new double[]{0, 0, 0}; }
    @Override
    public double[] speedPID() { return new double[]{0, 0, 0}; }
    @Override
    public double[] directionPID() { return new double[]{0, 0, 0}; }
    @Override
    public boolean useEncoders() { return false; }
}
