package org.firstinspires.ftc.teamcode._RobotCode.Schrodinger.MechanicalControllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SchrodingerFoundationGrabbers
{
    ////DEPENDENCIES////
    //Servos
    private Servo gripperR, gripperL;
    private DcMotor TM;

    ////VARIABLES////
    //Universal
    public static double servoPosPerDegrees = 0.0055; //TODO: find this coefficient
    private boolean gripperCurrentlyClosed = false;

    //Gripper Grabbing
    //TODO- Find max and min of gripper grabbers in degrees with closed as 180
    public static double grabUp = 20;
    public static double grabDown = 180;
    private double targetGrabPos;


    public SchrodingerFoundationGrabbers(Servo setGripperR, Servo setGripperL, DcMotor SetTM){
        gripperR = setGripperR;
        gripperL = setGripperL;
        TM = SetTM;
    }

    public void SetGrabberState(boolean closed){
        if(closed){
            gripperR.setPosition(0 + grabDown * servoPosPerDegrees);
            gripperL.setPosition(1 - grabDown * servoPosPerDegrees);
        }
        else {
            gripperR.setPosition(0 + grabUp * servoPosPerDegrees);
            gripperL.setPosition(1 - grabUp * servoPosPerDegrees);
        }
        gripperCurrentlyClosed = closed;
    }
    public void SwitchGrabberState(){
        if(gripperCurrentlyClosed) SetGrabberState(false);
        else SetGrabberState(true);
    }

    public void PrintTelemetry(Telemetry telemetry){
        telemetry.addLine("===FOUNDATION-GRABBERS===");
        telemetry.addData("fGrabberR pos ", gripperR.getPosition());
        telemetry.addData("fGrabberL pos ", gripperL.getPosition());
    }

    public void ExtendTape(){
        TM.setPower(0.5);
    }

    public void RetractTape(){
        TM.setPower(-0.5);
    }

    public void StopTape(){
        TM.setPower(0);
    }
}
