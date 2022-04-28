package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

import org.firstinspires.ftc.teamcode.Navigation.FieldState.Pose;

public class BaseRobot
{
   public enum FieldSide {RED,BLUE}
   public FieldSide fieldSide;
   public Pose robotPose;
   public boolean USE_PAYLOAD = false;
   public boolean USE_CHASSIS = false;
   public boolean USE_NAVIGATOR = false;

   public BaseRobot(FieldSide setSide, Pose setPose, boolean SET_USE_PAYLOAD, boolean SET_USE_CHASSIS, boolean SET_USE_NAVIGATOR){
      USE_PAYLOAD = SET_USE_PAYLOAD;
      USE_CHASSIS = SET_USE_CHASSIS;
      USE_NAVIGATOR = SET_USE_NAVIGATOR;
   }
}
