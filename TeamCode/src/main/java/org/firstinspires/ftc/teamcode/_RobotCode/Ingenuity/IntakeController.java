package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


class IntakeController {
    public IntakeController(double power_e) {
        power = power_e;
        active = false;
    }
    
    public IntakeController() {
        power = 0.1;
    }

    public void Init(OpMode opMode, String motor) {
        intake = opMode.hardwareMap.dcMotor.get(motor);
    }

    public void on(){
        intake.setPower(power);
        active = true;
    }

    public void on(double myPower){
        intake.setPower(myPower);
        active = true;
    }

    public void off(){
        intake.setPower(0);
        active = false;
    }




    public void toggle(){
       if(active){
           off();
       } else {
           on();
       }
    }

    public void setPowerOrion(double newPower) {
        power = newPower ;
    }

    public void setPower(double spower) throws BruhException {
        if(spower == 0)throw new BruhException("Power was set to '0'");
        power = spower;
    }
    
    private double power;
    private boolean active;

    private DcMotor intake;
}