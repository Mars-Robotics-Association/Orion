package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis;

public interface ChassisProfile
{
   double moveSpeed();
   double turnSpeed();
   boolean flipIMU();
   String[] motorNames();
   boolean[] flipMotors();
   double[] poseXYPID();
   double[] poseAnglePID();
   double[] speedPID();
   double[] trajectoryPID();
   boolean useEncoders();

}
