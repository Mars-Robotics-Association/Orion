package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;

public class BaseRobot
{
   public enum FieldSide {RED,BLUE}
   public enum LRSide{LEFT,RIGHT}
   public FieldSide fieldSide;
   public LRSide lrSide;
   public Pose robotPose;
   public boolean USE_PAYLOAD = false;
   public boolean USE_CHASSIS = false;
   public boolean USE_NAVIGATOR = false;
   ChassisProfile chassisProfile;

   public BaseRobot(FieldSide setSide, LRSide setLR, Pose setPose, boolean SET_USE_PAYLOAD, boolean SET_USE_CHASSIS, boolean SET_USE_NAVIGATOR){
      fieldSide = setSide;
      lrSide = setLR;
      USE_PAYLOAD = SET_USE_PAYLOAD;
      USE_CHASSIS = SET_USE_CHASSIS;
      USE_NAVIGATOR = SET_USE_NAVIGATOR;
   }

   public void setChassisProfile(ChassisProfile profile){chassisProfile=profile;}
   public ChassisProfile getChassisProfile(){return chassisProfile;}
   public FieldSide getFieldSide(){return fieldSide;}
   public LRSide getLrSide(){return lrSide;}
}
