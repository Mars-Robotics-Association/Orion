package org.firstinspires.ftc.teamcode._RobotCode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmControl
{
    public static double armMaxRots = 18;
    public static double armMinRots = 0;
    public static double encoderResolution = 537.7; //gobilda 19.2:1
    public static double encoderMultiplier = 1;
    public static boolean useEncoder = false;

    private OpMode opMode;
    private DcMotor arm;
    private Servo spinner;

    public ArmControl(OpMode setOpMode, DcMotor armMotor, Servo spinnerServo, boolean isUseEncoder, boolean reverseEncoder){
        arm = armMotor;
        spinner = spinnerServo;
        useEncoder = isUseEncoder;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode = setOpMode;

        if(reverseEncoder) encoderMultiplier = -1;
        else encoderMultiplier = 1;
    }

    public void SetArmRotation(double pos){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition((int)(encoderResolution*clamp(pos,armMinRots,armMaxRots)));
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ResetArmPos(){arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    public void LockArm(){
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void SetArmPower(double power){
        opMode.telemetry.addData("ARM POSITION", GetArmPos());
        if(power > 0 && GetArmPos() < armMinRots){
            if(useEncoder) SetArmRotation(armMinRots*encoderResolution);
            else arm.setPower(0);
            return;
        }
        else if(power < 0 && GetArmPos() > armMaxRots){
            if(useEncoder) SetArmRotation(armMaxRots*encoderResolution);
            else arm.setPower(0);
            return;
        }

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(power);
    }

    public double GetArmPos(){
        return arm.getCurrentPosition()*encoderMultiplier/encoderResolution;
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
