package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuatorProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.BlinkinController;

@Config
public class CuriosityTurretArm extends UniversalTurretIntakeArm
{
    ArmLeveller leveller;
    BlinkinController lights;

    public enum Alliance {RED,BLUE}
    public enum Strategy {TEAM,SHARED}
    public enum Tier {BOTTOM, MIDDLE, TOP, CAP}

    public Alliance alliance = Alliance.BLUE;
    public Strategy strategy = Strategy.TEAM;

    double turretSharedPlacementPos = -0.4; //shared hub
    double turretTeamPlacementPos = -0.4; //team hub

    //arm positions for each of the levels
    public static double armBottomPos = 0.1;
    public static double armMiddlePos = 0.2;
    public static double armTopPos = 0.34;
    public static double armCapPos = 0.34;

    int currentAutoIntakeTeir = 1; //what level to send the arm to when intaking


    public CuriosityTurretArm(OpMode setOpMode, BlinkinController setLights, EncoderActuatorProfile setArmProfile, EncoderActuatorProfile setTurretProfile, Servo intake, DistanceSensor intakeSensor, DistanceSensor resetSensor, TouchSensor armTouch, boolean reverseIntake) {
        super(setOpMode, setArmProfile, setTurretProfile, intake, intakeSensor, armTouch, reverseIntake);
        leveller = new ArmLeveller(opMode,resetSensor,intakeSensor,Arm());
        leveller.SetThread(new Thread(leveller));
        lights = setLights;
    }

    public void GoToTier(Tier tier){
        leveller.StopThread();
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

    @Override
    protected void IntakeFullAction(){
        lights.Lime();
        lights.SetCooldown(1);
    }

    //Automatically intakes taking to account tiers
    public void UpdateIntakeTiered(){
        if(currentAutoIntakeTeir == 0) {
            UpdateIntake(armBottomPos);
            opMode.telemetry.addData("Intake Tier", "BOTTOM");
        }
        else if(currentAutoIntakeTeir == 1) {
            UpdateIntake(armMiddlePos);
            opMode.telemetry.addData("Intake Tier", "MIDDLE");

        }
        else if(currentAutoIntakeTeir == 2) {
            UpdateIntake(armTopPos);
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
        leveller.StartResetArm();
    }

    public void StopAutoLeveller(){leveller.StopThread();}
}
