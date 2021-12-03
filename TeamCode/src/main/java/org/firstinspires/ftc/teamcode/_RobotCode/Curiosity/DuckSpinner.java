package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DuckSpinner
{
    double speedMultipler = 1;
    Servo spinner;

    //initializer
    public DuckSpinner(Servo setSpinner, double setSpeedMultipler){
        speedMultipler = setSpeedMultipler;
        spinner = setSpinner;
    }

    //Set the speed of the motor with the multiplier
    public void SetSpeed(double speed){spinner.setPosition(speed);}
    //Spin at full speed forwards
    public void Forwards(){SetSpeed(1);}
    //Cut power to the motor
    public void Stop(){SetSpeed(0.5);}
    //Spine at full speed backwards
    public void Reverse(){SetSpeed(0);}

}