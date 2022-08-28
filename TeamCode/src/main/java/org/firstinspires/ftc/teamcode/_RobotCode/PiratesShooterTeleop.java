package org.firstinspires.ftc.teamcode._RobotCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;

@TeleOp
public class PiratesShooterTeleop extends OpMode implements ControllerInputListener
{
    private ControllerInput controllerInput1;
    DcMotor motor1;
    DcMotor motor2;
    Servo loader;

    double servoLoadPos = 0.5;
    double shootSpeed = 1;

    @Override
    public void init() {
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        loader = hardwareMap.servo.get("loader");
    }

    @Override
    public void start(){
        loader.setPosition(0);
    }

    @Override
    public void loop() {
        controllerInput1.Loop();
        telemetry.addData("runtime:", getRuntime());
        telemetry.update();
    }

    @Override
    public void ButtonPressed(int id, ControllerInput.Button button) {
        telemetry.addLine("button pressed");
        if(button == ControllerInput.Button.LT){
            loader.setPosition(servoLoadPos);
        }
    }

    @Override
    public void ButtonHeld(int id, ControllerInput.Button button) {
        telemetry.addLine("button held: " + button.name());
        if(button == ControllerInput.Button.RT){
            telemetry.addLine("shooting!");
            motor1.setPower(shootSpeed);
            motor2.setPower(-shootSpeed);
        }
    }

    @Override
    public void ButtonReleased(int id, ControllerInput.Button button) {
        telemetry.addLine("button released");
        if(button == ControllerInput.Button.RT){
            motor1.setPower(0);
            motor2.setPower(0);
        }
        if(button == ControllerInput.Button.LT){
            loader.setPosition(0);
        }
    }
}
