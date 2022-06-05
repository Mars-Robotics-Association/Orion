package org.firstinspires.ftc.teamcode.TestingOpModes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;

@Config
@TeleOp(name = "motorServoTest")
public class motorServoTesting extends OpMode implements ControllerInputListener {

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;
    private Servo servo4;
    private Servo servo5;
    private Servo servo6;
    int currentmotor = 0;
    int currentservo = 0;

    private ControllerInput controllerInput1;

    @Override
    public void init(){
        motor1 = hardwareMap.dcMotor.get("motor 1");
        motor2 = hardwareMap.dcMotor.get("motor 2");
        motor3 = hardwareMap.dcMotor.get("motor 3");
        motor4 = hardwareMap.dcMotor.get("motor 4");
        servo1 = hardwareMap.servo.get("servo 1");
        servo2 = hardwareMap.servo.get("servo 2");
        servo3 = hardwareMap.servo.get("servo 3");
        servo4 = hardwareMap.servo.get("servo 4");
        servo5 = hardwareMap.servo.get("servo 5");
        servo6 = hardwareMap.servo.get("servo 6");

        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
    }

    @Override
    public void loop() {
        controllerInput1.Loop();
        if(gamepad1.right_trigger<0.1 && gamepad1.left_trigger<0.1){
            telemetry.addLine("STOP MOTORS");
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }

        telemetry.addData("current motor: ", currentmotor);
        telemetry.addData("current servo: ", currentservo);
        telemetry.addData("To spin motor: ", "left and right triggers");
        telemetry.addData("To move servo to extremes: ", "left and right bumpers");
        telemetry.addData("To move servo to middle position: ", "a");
        telemetry.addData("To switch motor: ", "up and down on dpad");
        telemetry.addData("To switch servo: ", "left and right on dpad");
        telemetry.addData("Make sure you are using ", "gamepad 1");
        telemetry.update();
    }

    @Override
    public void ButtonPressed(int id, ControllerInput.Button button) {
        //controller 1
        if(id == 1){
            //Colored buttons
            if(button == ControllerInput.Button.A);
            if(button == ControllerInput.Button.B);
            if(button == ControllerInput.Button.X);
            if(button == ControllerInput.Button.Y);
            //Bumpers
            if(button == ControllerInput.Button.LB);
            if(button == ControllerInput.Button.RB);
            //Triggers
            if(button == ControllerInput.Button.LT);
            if(button == ControllerInput.Button.RT);
            if(button == ControllerInput.Button.DUP) {
                currentmotor += 1;
                if (currentmotor > 4) currentmotor = 1;
            }
            if(button == ControllerInput.Button.DDOWN) {
                currentmotor -= 1;
                if (currentmotor < 1) currentmotor = 4;
            }
            if(button == ControllerInput.Button.DLEFT) {
                currentservo -= 1;
                if (currentservo < 1) currentservo = 6;
            }
            if(button == ControllerInput.Button.DRIGHT) {
                currentservo += 1;
                if (currentservo > 6) currentservo = 1;
            }
            //Joystick Buttons
            if(button == ControllerInput.Button.LJS);
            if(button == ControllerInput.Button.RJS);
        }
    }

    @Override
    public void ButtonHeld(int id, ControllerInput.Button button) {
        //controller 1
        if(id == 1){
            //Colored buttons
            if(button == ControllerInput.Button.A){
                switch (currentservo) {
                    case (1):
                        servo1.setPosition(0.5);
                        break;
                    case (2):
                        servo2.setPosition(0.5);
                        break;
                    case (3):
                        servo3.setPosition(0.5);
                        break;
                    case (4):
                        servo4.setPosition(0.5);
                        break;
                    case (5):
                        servo5.setPosition(0.5);
                        break;
                    case (6):
                        servo6.setPosition(0.5);
                        break;
                }
            }
            if(button == ControllerInput.Button.B);
            if(button == ControllerInput.Button.X);
            if(button == ControllerInput.Button.Y);
            //Bumpers
            if(button == ControllerInput.Button.LB){
                switch (currentservo) {
                    case (1):
                        servo1.setPosition(1);
                        break;
                    case (2):
                        servo2.setPosition(1);
                        break;
                    case (3):
                        servo3.setPosition(1);
                        break;
                    case (4):
                        servo4.setPosition(1);
                        break;
                    case (5):
                        servo5.setPosition(1);
                        break;
                    case (6):
                        servo6.setPosition(1);
                        break;
                }
            }
            if(button == ControllerInput.Button.RB){
                switch (currentservo) {
                    case (1):
                        servo1.setPosition(0);
                        break;
                    case (2):
                        servo2.setPosition(0);
                        break;
                    case (3):
                        servo3.setPosition(0);
                        break;
                    case (4):
                        servo4.setPosition(0);
                        break;
                    case (5):
                        servo5.setPosition(0);
                        break;
                    case (6):
                        servo6.setPosition(0);
                        break;
                }
            }
            //Triggers
            if(button == ControllerInput.Button.LT){
                switch (currentmotor) {
                    case (1):
                        motor1.setPower(gamepad1.left_trigger);
                        break;
                    case (2):
                        motor2.setPower(gamepad1.left_trigger);
                        break;
                    case (3):
                        motor3.setPower(gamepad1.left_trigger);
                        break;
                    case (4):
                        motor4.setPower(gamepad1.left_trigger);
                        break;
                }
            }
            if(button == ControllerInput.Button.RT){
                switch (currentmotor) {
                    case (1):
                        motor1.setPower(-gamepad1.right_trigger);
                        break;
                    case (2):
                        motor2.setPower(-gamepad1.right_trigger);
                        break;
                    case (3):
                        motor3.setPower(-gamepad1.right_trigger);
                        break;
                    case (4):
                        motor4.setPower(-gamepad1.right_trigger);
                        break;
                }
            }
            if(button == ControllerInput.Button.DUP);
            if(button == ControllerInput.Button.DDOWN);
            if(button == ControllerInput.Button.DLEFT);
            if(button == ControllerInput.Button.DRIGHT);
            //Joystick Buttons
            if(button == ControllerInput.Button.LJS);
            if(button == ControllerInput.Button.RJS);
        }
    }

    @Override
    public void ButtonReleased(int id, ControllerInput.Button button) {
        //controller 1
        if(id == 1){
            //Colored buttons
            if(button == ControllerInput.Button.A);
            if(button == ControllerInput.Button.B);
            if(button == ControllerInput.Button.X);
            if(button == ControllerInput.Button.Y);
            //Bumpers
            if(button == ControllerInput.Button.LB);
            if(button == ControllerInput.Button.RB);
            //Triggers
            if(button == ControllerInput.Button.LT);
            if(button == ControllerInput.Button.RT);
            if(button == ControllerInput.Button.DUP);
            if(button == ControllerInput.Button.DDOWN);
            if(button == ControllerInput.Button.DLEFT);
            if(button == ControllerInput.Button.DRIGHT);
            //Joystick Buttons
            if(button == ControllerInput.Button.LJS);
            if(button == ControllerInput.Button.RJS);
        }
    }
}
