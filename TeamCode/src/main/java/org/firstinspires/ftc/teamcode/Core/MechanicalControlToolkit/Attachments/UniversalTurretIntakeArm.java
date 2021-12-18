package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Class for controlling an arm with the potential of having a turret, arm, and servo intake
//Sensors Used:
//Distance sensor on the intake to detect collection
//Touch sensor at the bottom and top extremes

public class UniversalTurretIntakeArm
{
    EncoderActuatorProfile armProfile;
    EncoderActuator arm;
    //EncoderActuatorProfile turretProfile;
    //EncoderActuator turret;
    DistanceSensor intakeSensor;
    TouchSensor bottomSensor;
    public static double intakeMultiplier = 1;
    private double intakeState = 0;

    private OpMode opMode;
    private Servo intake;
    enum ArmState {Intaking, Storage, Placing}
    ArmState armState = ArmState.Storage;

    public UniversalTurretIntakeArm(OpMode setOpMode, EncoderActuatorProfile setArmProfile, EncoderActuatorProfile setTurretProfile, Servo setIntake, DistanceSensor setIntakeDetector, TouchSensor setBottomSensor, boolean reverseIntake){
        opMode = setOpMode;

        armProfile = setArmProfile;
        //turretProfile = setTurretProfile;
        intake = setIntake;

        arm = new EncoderActuator(opMode, armProfile);
        //turret = new EncoderActuator(opMode, turretProfile);

        intakeSensor = setIntakeDetector;
        bottomSensor = setBottomSensor;

        if(reverseIntake) intakeMultiplier = -1;
        else intakeMultiplier = 1;
    }

    public EncoderActuator Arm(){return arm;}
    //public EncoderActuator Turret(){return turret;}

    public void GoToZero(){
        arm.GoToPosition(0);
        //turret.GoToPosition(0);
    }

    public boolean ResetArmToTouchSensor(double power){
        if(bottomSensor.isPressed()){
            return true;
        }
        else{
            arm.SetPowerRaw(power);
            return false;
        }
    }

    public boolean IntakeRoutine(double intakeDistanceCM){
        opMode.telemetry.addData("ArmDist Sensor", intakeSensor.getDistance(DistanceUnit.CM));
        if(intakeSensor.getDistance(DistanceUnit.CM) < intakeDistanceCM){
            armState = ArmState.Storage;
            SetIntakeSpeed(0);
            GoToMax();
            return true;
        }
        else return false;
        /*if(armState == ArmState.Intaking){
            SetIntakeSpeed(1);
            if(intakeSensor.getDistance(DistanceUnit.CM) < intakeDistanceCM){
                armState = ArmState.Storage;
                SetIntakeSpeed(0);
                GoToMax();
                return true;
            }
        }
        else if(armState == ArmState.Storage){
            return true;
        }
        else if(armState == ArmState.Placing){
            armState = ArmState.Intaking;
            GoToZero();
            SetIntakeSpeed(1);
        }
        return false;*/
    }

    public void GoToMax(){
        arm.GoToMax();
        //turret.GoToMax();
        armState = ArmState.Storage;
    }

    public void SetIntakeSpeed(double speed){
        double clampedSpeed = clamp(speed, -1,1);
        double servoSpeed = (clampedSpeed * 0.5) + 0.5;
        intake.setPosition(servoSpeed);
    }

    public void CycleIntakeState(double intakeSpeed){
        opMode.telemetry.addLine("CYCLING INTAKE");
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
