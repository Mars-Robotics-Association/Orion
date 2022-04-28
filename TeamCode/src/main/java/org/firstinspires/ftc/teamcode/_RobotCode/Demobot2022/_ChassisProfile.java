package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;

class _ChassisProfile implements ChassisProfile
{
    public _ChassisProfile(){

    }

    @Override
    public boolean flipIMU() { return true; }
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
