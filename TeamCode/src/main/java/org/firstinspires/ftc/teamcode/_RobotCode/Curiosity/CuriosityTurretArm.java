package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

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
    double armMiddlePos = 0.175;
    double armTopPos = 0.25;
    double armCapPos = 0.3;

    public CuriosityTurretArm(OpMode setOpMode, EncoderActuatorProfile setArmProfile, EncoderActuatorProfile setTurretProfile, Servo intake, boolean reverseIntake) {
        super(setOpMode, setArmProfile, setTurretProfile, intake, reverseIntake);
    }

    public void GoToHubTier(Tier tier){

    }
}
