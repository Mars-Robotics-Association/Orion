package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;

public class CuriosityTurretArm extends UniversalTurretIntakeArm implements Runnable
{
    DistanceSensor resetSensor;
    double armSlowDistanceCM = 8;
    double armResetDistanceCM = 2;

    public enum Alliance {RED,BLUE}
    public enum Strategy {TEAM,SHARED}
    public enum Tier {BOTTOM, MIDDLE, TOP, CAP}

    public Alliance alliance = Alliance.BLUE;
    public Strategy strategy = Strategy.TEAM;

    double turretSharedPlacementPos = -0.4; //shared hub
    double turretTeamPlacementPos = -0.4; //team hub

    //arm positions for each of the levels
    double armBottomPos = 0.1;
    double armMiddlePos = 0.2;
    double armTopPos = 0.34;
    double armCapPos = 0.34;

    public static double armIntakeDist = 7;

    int currentAutoIntakeTeir = 1; //what level to send the arm to when intaking

    Thread thread;
    boolean threadRunning = true;



    public CuriosityTurretArm(OpMode setOpMode, EncoderActuatorProfile setArmProfile, EncoderActuatorProfile setTurretProfile, Servo intake, DistanceSensor intakeSensor, DistanceSensor setResetSensor, TouchSensor armTouch, boolean reverseIntake) {
        super(setOpMode, setArmProfile, setTurretProfile, intake, intakeSensor, armTouch, reverseIntake);
        resetSensor = setResetSensor;
    }

    public void GoToTier(Tier tier){
        if(tier == Tier.BOTTOM){
            Arm().GoToPosition(armBottomPos);
        }
        if(tier == Tier.MIDDLE){
            Arm().GoToPosition(armMiddlePos);
        }
        if(tier == Tier.TOP){
            Arm().GoToPosition(armTopPos);
        }
        if(tier == Tier.CAP){
            Arm().GoToPosition(armCapPos);
        }
    }

    //Automatically intakes taking to account tiers
    public void UpdateIntakeTiered(){
        if(currentAutoIntakeTeir == 0) {
            UpdateIntake(armIntakeDist,armBottomPos);
            opMode.telemetry.addData("Intake Tier", "BOTTOM");
        }
        else if(currentAutoIntakeTeir == 1) {
            UpdateIntake(armIntakeDist,armMiddlePos);
            opMode.telemetry.addData("Intake Tier", "MIDDLE");

        }
        else if(currentAutoIntakeTeir == 2) {
            UpdateIntake(armIntakeDist,armTopPos);
            opMode.telemetry.addData("Intake Tier", "TOP");

        }
    }

    public double GetCurrentAutoTierRotation(){
        if(currentAutoIntakeTeir == 0) {
            return armBottomPos;
        }
        else if(currentAutoIntakeTeir == 1) {
            return armMiddlePos;
        }
        else {
            return armTopPos;

        }
    }

    //Goes to the auto intake tier manually
    public void GoToAutoTier(){
        if(currentAutoIntakeTeir == 0) GoToTier(Tier.BOTTOM);
        if(currentAutoIntakeTeir == 1) GoToTier(Tier.MIDDLE);
        if(currentAutoIntakeTeir == 2) GoToTier(Tier.TOP);
    }

    public void AutoIntakeTierUp(){
        if(currentAutoIntakeTeir>=2) return;
        else currentAutoIntakeTeir++;
    }
    public void AutoIntakeTierDown(){
        if(currentAutoIntakeTeir<=0) return;
        else currentAutoIntakeTeir--;
    }

    public void ReturnToHomeAndIntakeWithSensor(){
        StartIntake(1);
        StartResetArm();
    }

    public void ResetArmLinear(){
        //go down until distance sensor detects floor
        while (resetSensor.getDistance(DistanceUnit.CM) > armResetDistanceCM && threadRunning){
            while (resetSensor.getDistance(DistanceUnit.CM) > armSlowDistanceCM && threadRunning) Arm().SetPowerRaw(0.8);//go fast
            Arm().SetPowerRaw(0.2); //slow
            opMode.telemetry.addData("Arm Reset Sensor Distance", resetSensor.getDistance(DistanceUnit.CM)+" CM");
            opMode.telemetry.update();
        }
        //reset the arm
        Arm().ResetToZero();
        Arm().SetPowerRaw(0);
    }

    public void StartResetArm(){
        thread.start();
    }

    public void StopThread(){
        threadRunning = false;
    }

    public void SetThread(Thread setThread) {thread=setThread;}

    @Override
    public void run() {
        threadRunning = true;
        ResetArmLinear();
    }

}
