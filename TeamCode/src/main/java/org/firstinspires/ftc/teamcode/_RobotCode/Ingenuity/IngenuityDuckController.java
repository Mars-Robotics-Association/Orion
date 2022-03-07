package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IngenuityDuckController {
    Servo duckController;
    double speed;

    public IngenuityDuckController(Servo setServo, double setSpeed) {
        duckController = setServo;
        speed = setSpeed / 200;
    }

    public void RedSide(){
        duckController.setPosition(0.5-speed);
    }
    public void Stop(){
        duckController.setPosition(0.5);
    }
    public void BlueSide(){
        duckController.setPosition(0.5+speed);
    }

}