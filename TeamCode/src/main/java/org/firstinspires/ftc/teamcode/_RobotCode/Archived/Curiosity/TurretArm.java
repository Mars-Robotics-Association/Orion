package org.firstinspires.ftc.teamcode._RobotCode.Archived.Curiosity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretArm
{
    public static double armMaxRots = 0.3;
    public static double armMinRots = 0;
    public static double armGearRatio = 24;
    public static double armEncoderResolution = 537.7; //gobilda 19.2:1
    public static double armEncoderMultiplier = 1;
    public static boolean useArmEncoder = false;

    public static double turretMaxRots = 1;
    public static double turretMinRots = -1;
    public static double turretGearRatio = 9;
    public static double turretEncoderResolution = 537.7; //gobilda 19.2:1
    public static double turretEncoderMultiplier = 1;
    public static boolean useTurretEncoder = false;

    private OpMode opMode;
    private DcMotor arm;
    private DcMotor turret;
    private Servo spinner;

    public TurretArm(OpMode setOpMode, DcMotor armMotor, DcMotor turretMotor, Servo spinnerServo, boolean isUseArmEncoder, boolean reverseArmEncoder, boolean isUseTurretEncoder, boolean reverseTurretEncoder){
        arm = armMotor;
        turret = turretMotor;
        spinner = spinnerServo;
        useArmEncoder = isUseArmEncoder;
        useTurretEncoder = isUseTurretEncoder;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode = setOpMode;

        if(reverseArmEncoder) armEncoderMultiplier = -1;
        else armEncoderMultiplier = 1;

        if(reverseTurretEncoder) turretEncoderMultiplier = -1;
        else turretEncoderMultiplier = 1;
    }

    //ARM
    //Sets the arm to go to a target rotation
    public void SetArmRotation(double pos){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition((int)(armEncoderResolution * clamp(pos,armMinRots,armMaxRots) * armGearRatio * armEncoderMultiplier));
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //Goes to arm's extreme
    public void SetArmToFurthestExtreme(){
        SetArmRotation(armMaxRots);
    }
    //Resets the arm's encoder
    public void ResetArmRot(){arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    //Sets arm's extreme
    public void ResetHighestRot(){armMaxRots = GetArmRotation();}
    //Locks the arm in place using the encoder
    public void LockArm(){
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //Sets the arms power and limits its position
    public void SetArmPower(double power){
        opMode.telemetry.addData("ARM POSITION", GetArmRotation());
        if(power > 0 && GetArmRotation() < armMinRots){
            if(useArmEncoder) SetArmRotation(armMinRots * armEncoderResolution * armGearRatio);
            else arm.setPower(0);
            return;
        }
        else if(power < 0 && GetArmRotation() > armMaxRots){
            if(useArmEncoder) SetArmRotation(armMaxRots * armEncoderResolution * armGearRatio);
            else arm.setPower(0);
            return;
        }

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(power);
    }
    //Returns the rotation of the arm
    public double GetArmRotation(){
        return arm.getCurrentPosition() * (armEncoderMultiplier / (armEncoderResolution * armGearRatio));
    }

    //Turret
    //Sets the turret to go to a target rotation
    public void SetTurretRotation(double pos){
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition((int)(turretEncoderResolution * clamp(pos,turretMinRots,turretMaxRots) * turretGearRatio / armEncoderMultiplier));
        turret.setPower(1);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //Resets the turret's encoder
    public void ResetTurretPos(){turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    //Locks the turret in place using the encoder
    public void LockTurret(){
        turret.setTargetPosition(turret.getCurrentPosition());
        turret.setPower(1);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //Sets the turrets power and limits its position
    public void SetTurretPower(double power){
        opMode.telemetry.addData("TURRET POSITION", GetTurretPos());
        if(power > 0 && GetTurretPos() < turretMinRots){
            if(useTurretEncoder) SetTurretRotation(turretMinRots * turretEncoderResolution * turretGearRatio);
            else turret.setPower(0);
            return;
        }
        else if(power < 0 && GetTurretPos() > turretMaxRots){
            if(useTurretEncoder) SetTurretRotation(turretMaxRots * turretEncoderResolution * turretGearRatio);
            else turret.setPower(0);
            return;
        }

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(power);
    }
    //Returns the rotation of the turret
    public double GetTurretPos(){
        return turret.getCurrentPosition() * (turretEncoderMultiplier / (turretEncoderResolution * turretGearRatio));
    }

    public void SetSpinnerSpeed(double speed){
        double clampedSpeed = clamp(speed, -1,1);
        double servoSpeed = (clampedSpeed * 0.5) + 0.5;
        spinner.setPosition(servoSpeed);
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
