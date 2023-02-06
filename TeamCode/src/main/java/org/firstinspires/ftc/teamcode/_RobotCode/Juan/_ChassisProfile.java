package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;

class _ChassisProfile implements ChassisProfile
{
    public _ChassisProfile(){}

    @Override
    public double moveSpeed() {return 1;}

    @Override
    public double turnSpeed() {return 1;}

    @Override
    public boolean flipIMU() { return false; }
    @Override
    public String[] motorNames() { return new String[]{"FR", "FL", "RR", "RL"}; }
    @Override
    public boolean[] flipMotors() {return new boolean[] {true,true,true,true}; }
    @Override
    public double[] poseXYPID() { return new double[]{0, 0, 0}; }
    @Override
    public double[] poseAnglePID() { return new double[]{0, 0, 0}; }
    @Override
    public double[] speedPID() { return new double[]{0, 0, 0}; }
    @Override
    public double[] trajectoryPID() { return new double[]{0, 0, 0}; }
    @Override
    public boolean useEncoders() { return false; }
}
