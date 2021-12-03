package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis;

public interface ChassisProfile
{
   boolean flipIMU();
   String[] motorNames();
   double[] headingPID();
   double[] speedPID();
   double[] directionPID();
   boolean useEncoders();

}
