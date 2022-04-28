package org.firstinspires.ftc.teamcode._RobotCode.Archived.Curiosity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;

class ArmLeveller implements Runnable
{
    OpMode opMode;
    DistanceSensor resetSensor;
    DistanceSensor intakeSensor;
    EncoderActuator arm;

    //levelling sensor
    double armSlowDistanceCM = 8;
    double armResetDistanceCM = 2;
    double armStorageLocation = 0.02;

    //intaking sensor
    double lowerArmDistanceCM = 10;

    //time buffer
    double lastTimeBlockDetected = 0;
    double timeToKeepDown = 1;

    Thread thread;
    boolean threadRunning = false;

    boolean resetArmQueued = false;
    boolean levelArmQueued = false;

    public ArmLeveller(OpMode setOpMode, DistanceSensor setResetSensor, DistanceSensor setIntakeSensor, EncoderActuator setArm){
        opMode = setOpMode;
        resetSensor = setResetSensor;
        intakeSensor = setIntakeSensor;
        arm = setArm;
    }

    public void ResetArmLinear(){
        resetArmQueued = false;
        //go down until distance sensor detects floor
        while (resetSensor.getDistance(DistanceUnit.CM) > armResetDistanceCM && threadRunning){
            while (resetSensor.getDistance(DistanceUnit.CM) > armSlowDistanceCM && threadRunning) arm.SetPowerRaw(1);//go fast
            arm.SetPowerRaw(0.2); //slow
            opMode.telemetry.addData("Arm Reset Sensor Distance", resetSensor.getDistance(DistanceUnit.CM)+" CM");
        }
        //reset the arm
        arm.ResetToZero();
        arm.SetPowerRaw(0);
    }

    /*public void LevelArm(){
        if(intakeSensor.getDistance(DistanceUnit.CM) < lowerArmDistanceCM) {
            arm.GoToPosition(0); //if something is close, go all the way down
            lastTimeBlockDetected = opMode.getRuntime();
        }
        else if(opMode.getRuntime() > lastTimeBlockDetected+timeToKeepDown) arm.GoToPosition(armStorageLocation); //only move up if its been a second
    }*/

    public void StartResetArm(){
        resetArmQueued = true;
        if(!threadRunning) thread.start();
    }

    /*public void StartLevelArmLoop(){
        levelArmQueued = true;
        if(!threadRunning) thread.start();
    }*/

    public void StopThread(){
        levelArmQueued = false;
        resetArmQueued = false;
        threadRunning = false;
    }

    public void SetThread(Thread setThread) {thread=setThread;}

    public boolean IsThreadRunning(){return threadRunning;}

    @Override
    public void run() {
        if(threadRunning) return;
        threadRunning = true;
        ResetArmLinear();
        threadRunning = false;
    }
}
