package org.firstinspires.ftc.teamcode.TestingOpModes;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;

public class Profile implements ChassisProfile {
    public Profile(){

    }

    @Override
    public boolean flipIMU() { return true; }
    @Override
    public String[] motorNames() { return new String[]{"FR", "FL", "RR", "RL"}; }
    @Override
    public boolean[] flipMotors() {return new boolean[] {true,true,true,true};}
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
