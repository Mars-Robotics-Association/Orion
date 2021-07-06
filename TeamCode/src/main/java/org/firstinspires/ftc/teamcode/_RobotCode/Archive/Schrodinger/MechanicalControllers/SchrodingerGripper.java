package org.firstinspires.ftc.teamcode._RobotCode.Schrodinger.MechanicalControllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SchrodingerGripper
{
    ////DEPENDENCIES////
    //Servos
    private Servo gripperR, gripperL, headingServo, rotationServoR, rotationServoL;

    ////VARIABLES////
    //Universal
    public static double servoPosPerDegrees = 0.0055; //TODO: find this coefficient
    private boolean gripperCurrentlyClosed = false;

    //Gripper Rotation
    //TODO- Find max and min of gripper rotation in degrees with intake as 0
    public static double rotationMax = 180;
    public static double rotationMin = 0;
    private double targetRotation;

    //Gripper Heading
    private double targetHeading;

    //Gripper Grabbing
    //TODO- Find max and min of gripper grabbers in degrees with closed as 0
    public static double grabOpen = 110;
    public static double grabClosed = 80;
    private double targetGrabPos;


    public SchrodingerGripper(Servo setGripperR, Servo setGripperL, Servo setHeadingServo, Servo setRotationServoR, Servo setRotationServoL){
        gripperR = setGripperR;
        gripperL = setGripperL;
        headingServo = setHeadingServo;
        rotationServoR = setRotationServoR;
        rotationServoL = setRotationServoL;

        ResetGripper();
    }

    public void SetTargetHeading(double degrees){
        headingServo.setPosition(0 + degrees * servoPosPerDegrees);
    }
    public void SetTargetRotation(double degrees){
        rotationServoR.setPosition(0 + degrees * servoPosPerDegrees);
        rotationServoL.setPosition(1 - degrees * servoPosPerDegrees);
    }
    public void ChangeTargetRotation(double degrees){
        rotationServoR.setPosition(rotationServoR.getPosition() + degrees * servoPosPerDegrees);
        rotationServoL.setPosition(rotationServoL.getPosition() - degrees * servoPosPerDegrees);
    }
    public void SetGripperState(boolean closed){
        if(closed){
            gripperR.setPosition(0 + grabClosed * servoPosPerDegrees);
            gripperL.setPosition(1 - grabClosed * servoPosPerDegrees);
        }
        else {
            gripperR.setPosition(0 + grabOpen * servoPosPerDegrees);
            gripperL.setPosition(1 - grabOpen * servoPosPerDegrees);
        }
        gripperCurrentlyClosed = closed;
    }
    public void SwitchGripperState(){
        if(gripperCurrentlyClosed) SetGripperState(false);
        else SetGripperState(true);
    }
    public void ResetGripper(){
        SetTargetHeading(90);
        SetTargetRotation(180);
        SetGripperState(false);
    }

    public void PrintTelemetry(Telemetry telemetry){
        telemetry.addLine("===GRIPPER===");
        telemetry.addData("gripperR pos ", gripperR.getPosition());
        telemetry.addData("gripperL pos ", gripperL.getPosition());
        telemetry.addData("headingGripper pos ", headingServo.getPosition());
        telemetry.addData("gripperRotationR pos ", rotationServoR.getPosition());
        telemetry.addData("gripperRotationL pos ", rotationServoL.getPosition());
    }
}
