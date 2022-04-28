package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;

/*
Class for controlling an arm, turret, and roller intake
Sensors used:
   -Distance sensor on the intake to detect collection
   -Distance sensor facing downwards to level arm and detect ground
Motors used:
   -DC motors w/ encoders for turret and arm
   -DC motor for intake
*/

@Config
public class ErasmusTurretArm
{
   //References
   protected OpMode opMode;
   ArmThread armThread;
   BlinkinController lights;
   BaseRobot baseRobot;

   //Motors
   EncoderActuatorProfile armProfile;
   public EncoderActuator arm;

   EncoderActuatorProfile turretProfile;
   public EncoderActuator turret;

   Servo intake;

   //Sensors
   public DistanceSensor intakeSensor;
   public DistanceSensor levelSensor;

   //Configuration
   public static double intakeMultiplier = 1;
   public static double armIntakeDist = 4;
   public static double armMoveCoefficient = 0.01;
   public static double turretMoveCoefficient = 0.1;

   //arm positions for each of the levels
   public static double armIntakePos = 0;
   public static double armBottomPos = 0.1;
   public static double armMiddlePos = 0.2;
   public static double armTopPos = 0.34;
   public static double armCapPos = 0.34;

   //turret positions
   double turretSharedPlacementPos = -0.4; //shared hub
   double turretTeamPlacementPos = -0.4; //team hub

   //levelling sensor
   double armSlowDistanceCM = 8;
   double armResetDistanceCM = 2;
   double armStorageLocation = 0.02;

   //States
   public int intakeState = 0;

   public enum Tier {COLLECT, BOTTOM, MIDDLE, TOP, CAP}
   public Tier currentAutoTier = Tier.MIDDLE;
   public Tier currentTier = Tier.COLLECT;


   public ErasmusTurretArm(OpMode setOpMode, BaseRobot setBaseRobot, BlinkinController setLights,
                           EncoderActuatorProfile setArmProfile, EncoderActuatorProfile setTurretProfile,
                           Servo setIntake, DistanceSensor setIntakeDetector, DistanceSensor setLevelSensor,
                           boolean reverseIntake)
   {
      opMode = setOpMode;
      baseRobot = setBaseRobot;
      lights = setLights;

      armProfile = setArmProfile;
      turretProfile = setTurretProfile;
      intake = setIntake;

      arm = new EncoderActuator(opMode, armProfile);
      turret = new EncoderActuator(opMode, turretProfile);

      intakeSensor = setIntakeDetector;
      levelSensor = setLevelSensor;

      armThread = new ArmThread(this);

      if(reverseIntake) intakeMultiplier = -1;
      else intakeMultiplier = 1;
   }

   //ARM AND TURRET MOVEMENT
   public void RotateTurret(double speed){
      turret.ChangeCurrentTargetRotation(turretMoveCoefficient*speed,speed);
   }
   public void RotateArm(double speed){
      arm.ChangeCurrentTargetRotation(armMoveCoefficient*speed,speed);
   }

   public double[] GetTurretArmBounds(double turretRot, double armRot){ //{turret min, turret max, arm min, arm max}, prioritizes arm
      return new double[]{0,0,0,0};
   }

   //MANAGE COLLECTION
   //what to do when the intake had collected a block
   protected void IntakeCollectAction(){
      lights.Lime();
      lights.SetCooldown(1);
   }

   //sets the speed of the intake within a -1 to 1 range
   public void SetIntakeSpeed(double speed){
      double clampedSpeed = clamp(speed, -1,1);
      double servoSpeed = (clampedSpeed * 0.5) + 0.5;
      intake.setPosition(servoSpeed);
   }

   //Returns the arm to the base position and starts intaking without doing any reset or levelling
   public void ReturnToHomeAndIntake(){
      StartIntake();
      GoToTier(Tier.COLLECT);
      turret.GoToPosition(0);
   }

   //Resets the arm and turns the intake on
   public void ResetArmAndIntake(){
      StartIntake();
      armThread.StartResetArm();
   }

   //Turns the intake on
   public void StartIntake(){
      SetIntakeSpeed(intakeMultiplier);
      intakeState = 1;
   }
   //Turns intake off
   public void StopIntake(){
      SetIntakeSpeed(0);
      intakeState = 2;
   }

   //Cycles between intake states: off, intaking, off, outaking
   public void CycleIntakeState(double intakeSpeed){
      opMode.telemetry.addLine("CYCLING INTAKE");
      if(intakeState == 0) SetIntakeSpeed(intakeSpeed);
      else if (intakeState == 1) SetIntakeSpeed(0);
      else if (intakeState == 2) SetIntakeSpeed(-intakeSpeed);
      else if (intakeState == 3) SetIntakeSpeed(0);
      intakeState ++;
      if(intakeState > 3) intakeState = 0;
   }


   //TIER MANAGEMENT
   //Moves the arm to specified tier
   public void GoToTier(Tier tier){
      armThread.StopThread();
      arm.GoToPosition(GetTierValue(tier));
      currentTier = tier;
   }

   //Moves to current auto-intake tier
   public void GoToAutoTier(){
      GoToTier(currentAutoTier);
   }

   //Returns a double of the arm's current tier rotation
   public double GetTierValue(Tier tier){
      if(tier == Tier.BOTTOM) return armBottomPos;
      else if(tier == Tier.MIDDLE) return armMiddlePos;
      else if(tier == Tier.TOP) return armTopPos;
      else if(tier == Tier.CAP) return armCapPos;
      else return armIntakePos;
   }

   //Sets the arm's auto position up a tier
   public void AutoIntakeTierUp(){
      if(currentAutoTier == Tier.BOTTOM) currentAutoTier = Tier.MIDDLE;
      else if(currentAutoTier == Tier.MIDDLE) currentAutoTier = Tier.TOP;
      else if(currentAutoTier == Tier.TOP) currentAutoTier = Tier.CAP;
   }

   //Sets the arm auto position down a tier
   public void AutoIntakeTierDown(){
      if(currentAutoTier == Tier.MIDDLE) currentAutoTier = Tier.BOTTOM;
      else if(currentAutoTier == Tier.TOP) currentAutoTier = Tier.MIDDLE;
      else if(currentAutoTier == Tier.CAP) currentAutoTier = Tier.TOP;
   }

   //UTILITY
   //Halts the thread of the arm
   public void StopArmThread(){
      armThread.StopThread();}

   //Returns the value in CMs of the intake sensor
   public double GetIntakeDistanceCM() {return intakeSensor.getDistance(DistanceUnit.CM);}

   //Clamps a value between a max and a min value
   public static double clamp(double val, double min, double max) {
      return Math.max(min, Math.min(max, val));
   }
}
