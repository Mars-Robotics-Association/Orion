package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DuckSpinner
{
    double speedMultipler = 1;
    DcMotor spinner;

    //initializer
    public DuckSpinner(DcMotor setSpinner, double setSpeedMultipler){
        speedMultipler = setSpeedMultipler;
        spinner = setSpinner;
    }

    //Set the speed of the motor with the multiplier
    public void SetSpeed(double speed){spinner.setPower(speed*speedMultipler);}
    //Spin at full speed forwards
    public void Forwards(){SetSpeed(1);}
    //Cut power to the motor
    public void Stop(){SetSpeed(0);}
    //Spine at full speed backwards
    public void Reverse(){SetSpeed(-1);}

}