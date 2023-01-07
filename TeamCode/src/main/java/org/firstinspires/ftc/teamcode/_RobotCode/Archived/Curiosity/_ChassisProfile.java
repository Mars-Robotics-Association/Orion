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
    public double moveSpeed() {
        return 1;
    }

    @Override
    public double turnSpeed() {
        return -1;
    }

    @Override
    public boolean flipIMU() { return false; }
    @Override
    public String[] motorNames() { return new String[]{"FR", "FL", "RR", "RL"}; }

    @Override
    public boolean[] flipMotors() {
        return new boolean[] {true,true,true,true};
    }

    @Override
    public double[] poseXYPID() { return new double[]{0, 0, 0}; }

    @Override
    public double[] poseAnglePID() {
        return new double[]{0,0,0};
    }

    @Override
    public double[] speedPID() { return new double[]{0, 0, 0}; }
    @Override
    public double[] trajectoryPID() { return new double[]{0, 0, 0}; }
    @Override
    public boolean useEncoders() { return false; }
}
