package org.firstinspires.ftc.teamcode._RobotCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;

@TeleOp
@Config
public class PiratesShooterTeleop extends OpMode implements ControllerInputListener
{
    private ControllerInput controllerInput1;
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    Servo loader;
    ElapsedTime timer;
    BlinkinController lights;

    double servoLoadPos = 0.5;
    double shootSpeed = 1;
    double speed1;
    int prevPosition1;
    double speed2;
    int prevPosition2;
    int prevPosition3;
    double speed3;
    int prevPosition4;
    double speed4;

    //motor rpm checking
    int checkInterval = 200; //ms between checking for target rpm
    double motor1RPM = 0;
    double motor2RPM = 0;
    int motor1PreviousPos;
    int motor2PreviousPos;

    @Override
    public void init() {
        //init controller input system
        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
        //init motors
        setUpMotors();
        //start timer
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        //init lights
        lights = new BlinkinController(this);
    }

    //sets up the motors
    private void setUpMotors() {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        loader = hardwareMap.servo.get("loader");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        checkInterval = 200;
        prevPosition1 = motor1.getCurrentPosition();
        prevPosition2 = motor2.getCurrentPosition();
        prevPosition3 = motor3.getCurrentPosition();
        prevPosition4 = motor4.getCurrentPosition();
        speed1=0;
        speed2=0;
        speed3=0;
        speed4=0;
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        lights = new BlinkinController(this);
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
            speed1 = Math.abs(speed1);
            speed2 = (double) (motor2.getCurrentPosition() - prevPosition2) / timer.time();
            speed2 = (speed2*1000*60)/28;
            speed2 = Math.abs(speed2);
            speed3 = (double) (motor3.getCurrentPosition() - prevPosition3) / timer.time();
            speed3 = (speed3*1000*60)/28;
            speed3 = Math.abs(speed3);
            speed4 = (double) (motor4.getCurrentPosition() - prevPosition4) / timer.time();
            speed4 = (speed4*1000*60)/28;
            speed4 = Math.abs(speed4);
            prevPosition1 = motor1.getCurrentPosition();
            prevPosition2 = motor2.getCurrentPosition();
            prevPosition3 = motor3.getCurrentPosition();
            prevPosition4 = motor4.getCurrentPosition();
            timer.reset();
        }
        //28 ticks per revolution
        telemetry.addData("Motor 1 Rev per min", speed1);
        telemetry.addData("Motor 2 Rev per min", speed2);
        telemetry.addData("Motor 3 Rev per min", speed3);
        telemetry.addData("Motor 4 Rev per min", speed4);
        if (speed1 > 2500 && speed2 > 2500) {
            telemetry.addData("READY TO FIRE", "!!!!!");
            lights.Green();
        } else if (speed1 > 0 || speed2 > 0 || speed3 > 0 || speed4 > 0) {
            lights.Red();
        } else {
            lights.SetPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
        }
    }

//    //TODO comment this
//    private void updateMotors() {
//        if (timer.time() > checkInterval) {
//            //28 ticks per revolution
//            motor1RPM = (double) (motor1.getCurrentPosition() - motor1PreviousPos) / timer.time();
//            motor1RPM = (motor1RPM *1000*60)/28;
//            motor1RPM = Math.abs(motor1RPM);
//            motor2RPM = (double) (motor2.getCurrentPosition() - motor2PreviousPos) / timer.time();
//            motor2RPM = (motor2RPM *1000*60)/28;
//            motor2RPM = Math.abs(motor2RPM);
//            motor1PreviousPos = motor1.getCurrentPosition();
//            motor2PreviousPos = motor2.getCurrentPosition();
//            timer.reset();
//        }
//        telemetry.addData("Motor1 Rev per min", motor1RPM);
//        telemetry.addData("Motor2 Rev per min", motor2RPM);
//    }

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
            motor3.setPower(shootSpeed);
            motor4.setPower(-shootSpeed);
            motor1.getCurrentPosition();
        }
    }

    @Override
    public void ButtonReleased(int id, ControllerInput.Button button) {
        telemetry.addLine("button released");
        if(button == ControllerInput.Button.RT){
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }
        if(button == ControllerInput.Button.LT){
            loader.setPosition(0);
        }
    }

}