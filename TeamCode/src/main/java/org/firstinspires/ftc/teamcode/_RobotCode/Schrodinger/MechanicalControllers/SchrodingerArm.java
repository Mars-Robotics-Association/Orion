package org.firstinspires.ftc.teamcode._RobotCode.Schrodinger.MechanicalControllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SchrodingerArm
{
    ////DEPENDENCIES////
    //Motors
    public DcMotor armMotor;
    //Servos
    private Servo armServoR, armServoL;

    ////VARIABLES////

    //Arm Rotation
    //TODO- Find max and min of arm in degrees with intake as 0
    public static double armMotorPosPerDegrees = 0.017;
    public static double armRotationMax = 180;
    public static double armRotationMin = 0;
    private double targetRotation;

    //Arm Extension
    //TODO- Find max and min of arm extension
    public static double extensionServoPosPerInch = 10;
    public static double armExtensionMax = 10;
    public static double armExtensionMin = 0;
    private double targetExtension;

    public SchrodingerArm(DcMotor setArmMotor, Servo setArmServoR, Servo setArmServoL){
        armMotor = setArmMotor;
        armServoR = setArmServoR;
        armServoL = setArmServoL;

        ResetArmMotor();
    }

    public void ResetArmMotor() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        SetTargetRotation(armRotationMin);
        armMotor.setPower(1);
    }

    public void SetTargetRotation(double rotation){
        armMotor.setTargetPosition((int) (rotation/armMotorPosPerDegrees));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void SetTargetExtension(double inches){
        armServoL.setPosition(0 + inches * extensionServoPosPerInch);
        armServoR.setPosition(1 - inches * extensionServoPosPerInch);
    }
    public void ChangeRotation(double speed){
        armMotor.setTargetPosition((int) ((int) (armMotor.getTargetPosition()/armMotorPosPerDegrees + speed) * armMotorPosPerDegrees));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ChangeExtension(double speed){
        armServoL.setPosition(speed);
        armServoR.setPosition(speed);
    }
    public void ArmToZero(){
        SetTargetRotation(armRotationMin);
        //SetTargetExtension(armExtensionMin);
    }

    public void PrintTelemetry(Telemetry telemetry){
        telemetry.addLine("===ARM===");
        telemetry.addData("armMotor pos ", armMotor.getCurrentPosition() * armMotorPosPerDegrees);
        telemetry.addData("armMotor target ", armMotor.getTargetPosition());
        telemetry.addData("armServoR pos ", armServoR.getPosition());
        telemetry.addData("armServoL pos ", armServoL.getPosition());
    }
}
