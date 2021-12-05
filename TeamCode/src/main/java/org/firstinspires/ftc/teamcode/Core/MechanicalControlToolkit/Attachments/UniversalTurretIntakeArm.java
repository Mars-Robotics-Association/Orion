package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

//Class for controlling an arm with the potential of having a turret, arm, and servo intake
//Sensors Used:
//Distance sensor on the intake to detect collection
//Touch sensor at the bottom and top extremes

public class UniversalTurretIntakeArm
{
    EncoderActuatorProfile armProfile;
    EncoderActuator arm;
    EncoderActuatorProfile turretProfile;
    EncoderActuator turret;
    public static double intakeMultiplier = 1;
    private double intakeState = 0;

    private OpMode opMode;
    private Servo intake;

    public UniversalTurretIntakeArm(OpMode setOpMode, EncoderActuatorProfile setArmProfile, EncoderActuatorProfile setTurretProfile, Servo setIntake, boolean reverseIntake){
        opMode = setOpMode;

        armProfile = setArmProfile;
        turretProfile = setTurretProfile;
        intake = setIntake;

        arm = new EncoderActuator(opMode, armProfile);
        turret = new EncoderActuator(opMode, turretProfile);

        if(reverseIntake) intakeMultiplier = -1;
        else intakeMultiplier = 1;
    }

    public EncoderActuator Arm(){return arm;}
    public EncoderActuator Turret(){return turret;}

    public void GoToZero(){
        arm.GoToPosition(0);
        turret.GoToPosition(0);
    }

    public void GoToMax(){
        arm.GoToMax();
        turret.GoToMax();
    }

    public void SetIntakeSpeed(double speed){
        double clampedSpeed = clamp(speed, -1,1);
        double servoSpeed = (clampedSpeed * 0.5) + 0.5;
        intake.setPosition(servoSpeed);
    }

    public void CycleIntakeState(double intakeSpeed){
        if(intakeState == 0) SetIntakeSpeed(intakeSpeed);
        else if (intakeState == 1) SetIntakeSpeed(0);
        else if (intakeState == 2) SetIntakeSpeed(-intakeSpeed);
        else if (intakeState == 3) SetIntakeSpeed(0);
        intakeState ++;
        if(intakeState > 3) intakeState = 0;
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
