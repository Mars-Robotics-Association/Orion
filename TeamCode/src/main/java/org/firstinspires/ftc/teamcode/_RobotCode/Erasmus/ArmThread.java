package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

class ArmThread implements Runnable
{
   ErasmusTurretArm turretArm;
   Thread thread;

   boolean threadRunning = false;

   boolean startResetArm = false;
   boolean updateAutoIntake = false;
   boolean updateArmLevelling = false;

   public ArmThread(ErasmusTurretArm setTurretArm){
      turretArm = setTurretArm;
      thread = new Thread(this);
   }

   public void SetThread(Thread setThread){
      thread = setThread;
   }

   @Override
   public void run() {
      if(threadRunning) return;
      threadRunning = true;

      //run until told to stop, cycling through each of the available tasks
      while (threadRunning){
         if(startResetArm) ResetArmLinear();
         if(updateAutoIntake) UpdateAutoIntake();
         if(updateArmLevelling) UpdateArmLevelling();
      }
      StopThread();
   }

   //Stops the thread completely
   public void StopThread(){
      threadRunning = false;
      turretArm.arm.SetPowerRaw(0);
      turretArm.StopIntake();
      turretArm.turret.SetPowerRaw(0);
   }

   //STARTING / STOPPING FUNCTIONS
   public void StartResetArm(){
      startResetArm = true;
      thread.start();
   }
   public void StartAutoIntake(){
      updateAutoIntake = true;
      turretArm.StartIntake();
      thread.start();
   }
   public void StartAutoLevelling(){
      updateArmLevelling = true;
      thread.start();
   }

   public void StopAutoIntake(){updateAutoIntake = false;}
   public void StopAutoLevelling(){updateArmLevelling = false;}

   //LINEAR FUNCTIONS
   //Resets the arm based off the the distance sensor
   public void ResetArmLinear(){
      startResetArm = false;
      //go down until distance sensor detects floor
      DistanceSensor levelSensor = turretArm.levelSensor;
      while (levelSensor.getDistance(DistanceUnit.CM) > turretArm.armResetDistanceCM && threadRunning){
         while (levelSensor.getDistance(DistanceUnit.CM) > turretArm.armSlowDistanceCM && threadRunning) turretArm.arm.SetPowerRaw(1);//go fast
         turretArm.arm.SetPowerRaw(0.2); //slow
      }
      //reset the arm only if actually reached end of motion and wasn't canceled
      if(threadRunning) turretArm.arm.ResetToZero();
      //stop the arm regardless
      turretArm.arm.SetPowerRaw(0);
   }

   //Updates the arms intake, raising the arm and stopping the intake when a block is collected
   public void UpdateAutoIntake(){
      //if thread is stopped, stop loop and return
      if(!threadRunning){
         return;
      }
      //if currently intaking AND something is detected
      if(turretArm.intakeState == 1 && turretArm.intakeSensor.getDistance(DistanceUnit.CM) < turretArm.armIntakeDist){
         turretArm.StopIntake(); //stops the intake
         turretArm.IntakeCollectAction(); //activates the action to perform when something is collected
         turretArm.GoToAutoTier(); //goes to automatic tier
      }
   }

   //Keeps the arm level with the ground using a proportional coefficient
   public void UpdateArmLevelling(){

   }
}
