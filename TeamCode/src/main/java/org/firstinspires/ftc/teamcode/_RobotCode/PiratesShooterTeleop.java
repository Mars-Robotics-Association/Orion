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
    Servo loader;
    ElapsedTime timer;
    BlinkinController lights;

    public static double servoLoadPos = 0.5;
    public static double shootSpeed = 1;

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
        loader = hardwareMap.servo.get("loader");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1PreviousPos = motor1.getCurrentPosition();
        motor2PreviousPos = motor2.getCurrentPosition();
    }

    @Override
    public void start(){
        loader.setPosition(0);
    }

    @Override
    public void loop() {
        controllerInput1.loop();

        telemetry.addData("runtime:", getRuntime());

        updateMotors();
        updateLights();

        printInstructions();

        telemetry.update();
    }

    //prints instructions to telemetry
    private void printInstructions(){
        telemetry.addLine();
        telemetry.addLine("----INSTRUCTIONS----");
        telemetry.addData("Spin up shooter: ", "Right Trigger");
        telemetry.addData("Fire: ", "Left Trigger");
        telemetry.addLine("Wait until lights are green to shoot!");
        telemetry.addLine();
    }

    private void updateLights() {
        if (motor1RPM > 2500 || motor2RPM > 2500) {
            telemetry.addData("READY TO FIRE", "!!!!!");
            lights.Green();
        } else if (motor1RPM > 0 || motor2RPM > 0) {
            lights.Red();
        } else {
            lights.SetPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
        }
    }

    //TODO comment this
    private void updateMotors() {
        if (timer.time() > checkInterval) {
            //28 ticks per revolution
            motor1RPM = (double) (motor1.getCurrentPosition() - motor1PreviousPos) / timer.time();
            motor1RPM = (motor1RPM *1000*60)/28;
            motor1RPM = Math.abs(motor1RPM);
            motor2RPM = (double) (motor2.getCurrentPosition() - motor2PreviousPos) / timer.time();
            motor2RPM = (motor2RPM *1000*60)/28;
            motor2RPM = Math.abs(motor2RPM);
            motor1PreviousPos = motor1.getCurrentPosition();
            motor2PreviousPos = motor2.getCurrentPosition();
            timer.reset();
        }
        telemetry.addData("Motor1 Rev per min", motor1RPM);
        telemetry.addData("Motor2 Rev per min", motor2RPM);
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