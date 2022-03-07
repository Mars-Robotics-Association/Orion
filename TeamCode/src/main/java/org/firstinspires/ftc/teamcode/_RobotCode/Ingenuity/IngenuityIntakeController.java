package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class IngenuityIntakeController {
    public IngenuityIntakeController() { power = 0.5; }

    public IngenuityIntakeController(Servo getServo) {
        intake = getServo;
    }

    public void on(){
        intake.setPosition(power);
        active = true;
    }

    public void on(double setPower){
        intake.setPosition(setPower);
        active = true;
    }

    public void off(){
        intake.setPosition(0.5);
        active = false;
    }

    public void toggle(){
       if(active){
           off();
       } else {
           on();
       }
    }

    public void setPower(double setPower) { power = setPower; }
    
    private double power;
    private boolean active = false;

    private Servo intake;
}