package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis;

public class ChassisProfile
{
   public boolean flipIMU = false;
   public String[] motorNames = {"FR","FL","RR","RL"};
   public double[] headingPID = {0,0,0};
   public double[] speedPID = {0,0,0};
   public double[] directionPID = {0,0,0};
   public boolean useEncoders = false;

}
