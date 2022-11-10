package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis;

public interface ChassisProfile
{
   double moveSpeed();//calibrate these speeds to work when driving with gamepad
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
