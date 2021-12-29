package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;

class CuriosityTurretArm extends UniversalTurretIntakeArm
{
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
    double armTopPos = 0.3;
    double armCapPos = 0.34;

    public static double armIntakeDist = 6;

    int currentAutoIntakeTeir = 1; //what level to send the arm to when intaking



    public CuriosityTurretArm(OpMode setOpMode, EncoderActuatorProfile setArmProfile, EncoderActuatorProfile setTurretProfile, Servo intake, DistanceSensor intakeSensor, TouchSensor armTouch, boolean reverseIntake) {
        super(setOpMode, setArmProfile, setTurretProfile, intake, intakeSensor, armTouch, reverseIntake);
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

}
