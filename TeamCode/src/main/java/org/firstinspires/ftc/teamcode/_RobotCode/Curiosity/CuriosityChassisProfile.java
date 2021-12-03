package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;

class CuriosityChassisProfile extends ChassisProfile
{
    public CuriosityChassisProfile(){
        flipIMU = false;
        motorNames = new String[]{"FR", "FL", "RR", "RL"};
        headingPID = new double[]{0, 0, 0};
        speedPID = new double[]{0, 0, 0};
        directionPID = new double[]{0, 0, 0};
        useEncoders = false;
    }
}
