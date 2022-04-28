package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DuckSpinner
{
    double speedMultipler = 1;
    DcMotor spinner;
    int duckSpinnerState = 0;

    //initializer
    public DuckSpinner(DcMotor setSpinner, double setSpeedMultipler){
        speedMultipler = setSpeedMultipler;
        spinner = setSpinner;
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Set the speed of the motor with the multiplier
    public void SetSpeed(double speed){spinner.setPower(speed);}
    //Spin at full speed forwards
    public void Blue(){SetSpeed(1);}
    //Cut power to the motor
    public void Stop(){SetSpeed(0);}
    //Spine at full speed backwards
    public void Red(){SetSpeed(-1);}
    //gradual increase
    public void GradSpin(boolean isBlue, double multiplier, double maxSpeed, OpMode opmode){
        int m = 1;
        if(!isBlue)m=-1;
        double startTime = opmode.getRuntime();
        double speed = 0;
        while(speed<maxSpeed){
            speed = (opmode.getRuntime()-startTime)*multiplier;
            if (speed>1)speed=1;
            if(speed>maxSpeed)speed = maxSpeed;
            SetSpeed(speed*m);
        }
        SetSpeed(0);
    }
    //Toggle
    public void CycleSpinning(){
        if(duckSpinnerState > 3) duckSpinnerState = 0;
        if(duckSpinnerState==0) Blue();
        else if(duckSpinnerState==1) Stop();
        else if(duckSpinnerState==2) Red();
        else Stop();
        duckSpinnerState++;
    }

}