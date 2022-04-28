package org.firstinspires.ftc.teamcode.Navigation.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

class ChassisFunctions
{
   ////Dependencies////
   OpMode opMode;
   MecanumChassis chassis;
   FreightFrenzyNavigator navigator;

   ////configuration variables////
   //Basic
   protected double whiteThreshold = 100;
   protected double wallStopDistance = 14;
   public static double turnCoefficient = 0.02;



   public ChassisFunctions(OpMode setOpMode, MecanumChassis setChassis, FreightFrenzyNavigator setNavigator){
      opMode = setOpMode;
      chassis = setChassis;
      navigator = setNavigator;
   }

   ////UTILITY / GENERAL FUNCTIONS////
   //Whether a timer has run out
   public boolean IsTimeUp(double startTime, double runTime){ return opMode.getRuntime()<startTime+runTime && navigator.IsNavigatorRunning(); }
   //If within distance sensor range
   boolean CheckDistance(DistanceSensor sensor,double distance){
      if(!navigator.IsNavigatorRunning()) return false;
      double dist = sensor.getDistance(DistanceUnit.CM);
      if(!(dist>0)) return false;//if distance is not greater than 0
      if(dist<distance) return true;
      else return false;
   }
   //Wait for a period of time
   public void Wait(double time){
      double startTime = opMode.getRuntime();
      while (IsTimeUp(startTime,time)){}
   }

   //Drive for a period of time
   public void DriveForTime(double angle, double speed, double turnOffset, double time){
      double startTime = opMode.getRuntime();
      while (IsTimeUp(startTime,time)) chassis.RawDrive(angle,speed,turnOffset);
      chassis.RawDrive(0,0,0);
   }

   //Drive for a period of time
   public void DriveForTimeToAngle(double driveAngle, double speed, double turnAngle, double coefficient, double time){
      double startTime = opMode.getRuntime();
      while (IsTimeUp(startTime,time)) chassis.RawDriveTurningTowards(driveAngle,speed,turnAngle,coefficient);
      chassis.RawDrive(0,0,0);
   }

   //Drive until duck sensor detects distance
   public void DriveForDistance(DistanceSensor sensor, double angle, double speed, double turnOffset, double distance){
      while (CheckDistance(sensor,distance)) chassis.RawDrive(angle,speed,turnOffset);
      chassis.RawDrive(0,0,0);
   }

   //Goes to the wall at an angle. Stops when in contact with wall
   public void GoToWall(double angle, double speed){
      while (!CheckDistance(navigator.portDist,wallStopDistance) && !CheckDistance(navigator.starboardDist,wallStopDistance))
         chassis.RawDrive(angle, speed, 0);
      chassis.RawDrive(0,0,0);
   }
   public void GoToWall(double speed){
      GoToWall(90*(-navigator.CalculateSideMultiplier()),speed);
   }

   //Goes to the wall at an angle. Stops when in contact with wall
   public void GoToWallTurning(double driveAngle, double speed, double facingAngle, double coefficient){
      while (!CheckDistance(navigator.portDist,wallStopDistance) && !CheckDistance(navigator.starboardDist,wallStopDistance) && navigator.IsNavigatorRunning())
         chassis.RawDriveTurningTowards(driveAngle, speed, facingAngle,coefficient);
      chassis.RawDrive(0,0,0);
   }

   //Wall follows at specified speed, which also determines direction.
   public void WallFollowForTime(double speed, double time){
      double turnOffset = 0.02*speed*navigator.CalculateSideMultiplier();
      double startTime = opMode.getRuntime();
      while (IsTimeUp(startTime,time)) chassis.RawDrive(0,speed,turnOffset);
      chassis.RawDrive(0,0,0);
   }

   //Wall follows at specified speed, which also determines direction .
   public void WallFollowForDuckDistance(double speed, double distance){
      double turnOffset = 0.02*speed*navigator.CalculateSideMultiplier();
      while (!CheckDistance(navigator.duckDistance,distance) && navigator.IsNavigatorRunning()) chassis.RawDrive(0,speed,turnOffset);
      chassis.RawDrive(0,0,0);
   }

   //Wall follows until white is detected by colorSensor. Must be called every loop().
   public void WallFollowToWhite(double speed, double angle){
      double turnOffset = 0.02*speed*navigator.CalculateSideMultiplier();
      if(angle>0) turnOffset*=-1;
      while (navigator.colorSensor.alpha() < whiteThreshold && navigator.IsNavigatorRunning()) chassis.RawDrive(angle,speed,turnOffset);
      chassis.RawDrive(0,0,0);

   }

   //Turns to an angle
   public void TurnToAngle(double angle, double speed){
      while (!chassis.InWithinRangeOfAngle(angle,5) && navigator.IsNavigatorRunning()) {
         chassis.TurnTowardsAngle(angle, speed, turnCoefficient);
         opMode.telemetry.addData("Robot Angle", chassis.GetImu().GetRobotAngle());
         opMode.telemetry.update();
      }
   }
}
