package org.firstinspires.ftc.teamcode._RobotCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;

@TeleOp
public class PiratesShooterTeleop extends OpMode implements ControllerInputListener
{
    private ControllerInput controllerInput1;
    DcMotor motor1;
    DcMotor motor2;
    Servo loader;
    ElapsedTime timer;

    double servoLoadPos = 0.5;
    double shootSpeed = 1;
    double speed1;
    int checkInterval;
    int prevPosition1;
    double speed2;
    int prevPosition2;

    @Override
    public void init() {
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        loader = hardwareMap.servo.get("loader");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        checkInterval = 200;
        prevPosition1 = motor1.getCurrentPosition();
        prevPosition2 = motor2.getCurrentPosition();
        speed1=0;
        speed2=0;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();

    }

    @Override
    public void start(){
        loader.setPosition(0);
    }

    @Override
    public void loop() {
        controllerInput1.Loop();
        telemetry.addData("runtime:", getRuntime());
        if (timer.time() > checkInterval) {
            speed1 = (double) (motor1.getCurrentPosition() - prevPosition1) / timer.time();
            speed1 = (speed1*1000*60)/28;
            speed2 = (double) (motor2.getCurrentPosition() - prevPosition2) / timer.time();
            speed2 = (speed2*1000*60)/28;
            prevPosition1 = motor1.getCurrentPosition();
            prevPosition2 = motor2.getCurrentPosition();
            timer.reset();
        }
        //537.7 ticks per revolution
        //actually 28
        telemetry.addData("Motor1 Rev per min", speed1);
        telemetry.addData("Motor2 Rev per min", speed2);
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
            motor1.getCurrentPosition();
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