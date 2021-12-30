package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

class IngenuityDuckController {
    Servo duckController;

    public IngenuityDuckController(Servo setServo) {
        duckController = setServo;
    }

    public void RedSide(){
        duckController.setPosition(0);
    }
    public void Stop(){
        duckController.setPosition(0.5);
    }
    public void BlueSide(){
        duckController.setPosition(1);
    }

}