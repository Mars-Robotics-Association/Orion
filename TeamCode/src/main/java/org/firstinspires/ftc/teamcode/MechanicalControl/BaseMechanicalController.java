package org.firstinspires.ftc.teamcode.MechanicalControl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseMechanicalController
{
    //Motors
    DcMotor motor1;

    //Servos
    Servo servo1;

    public void Init(DcMotor setMotor1, Servo setServo1){
        motor1 = setMotor1;
        servo1 = setServo1;
    }

    public void RunMotor(){
        motor1.setPower(1);
    }

    public void ServoToExtreme(){
        servo1.setPosition(1);
    }
}
