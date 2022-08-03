package org.firstinspires.ftc.teamcode.TestingOpModes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInputListener;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.ServoArray;

@Config
@TeleOp(name = "motorServoTest")
public class motorServoTesting extends OpMode implements ControllerInputListener {

    private DCMotorArray motors;
    private ServoArray servos;

    int currentmotor = 0;
    int currentservo = 0;
    int numMotors = 1;

    private ControllerInput controllerInput1;

    @Override
    public void init(){
        DcMotor motor1 = hardwareMap.dcMotor.get("motor 1");
        DcMotor motor2 = hardwareMap.dcMotor.get("motor 2");
        DcMotor motor3 = hardwareMap.dcMotor.get("motor 3");
        DcMotor motor4 = hardwareMap.dcMotor.get("motor 4");
        Servo servo1 = hardwareMap.servo.get("servo 1");
        Servo servo2 = hardwareMap.servo.get("servo 2");
        Servo servo3 = hardwareMap.servo.get("servo 3");
        Servo servo4 = hardwareMap.servo.get("servo 4");
        Servo servo5 = hardwareMap.servo.get("servo 5");
        Servo servo6 = hardwareMap.servo.get("servo 6");

        motors = new DCMotorArray(new DcMotor[]{motor1,motor2,motor3,motor4}, new double[]{1,1,1,1}, false);
        motors.RunWithoutEncodersMode();

        servos = new ServoArray(new Servo[]{servo1,servo2,servo3,servo4,servo5,servo6}, new double[]{.5,.5,.5,.5,.5,.5});
        servos.GoToPosition(0.5);

        controllerInput1 = new ControllerInput(gamepad1, 1);
        controllerInput1.addListener(this);
    }

    @Override
    public void loop() {
        controllerInput1.Loop();
        if(gamepad1.right_trigger<0.1 && gamepad1.left_trigger<0.1){
            telemetry.addLine("STOP MOTORS");
            motors.SetPowers(0);
        }

        telemetry.addData("current motor: ", currentmotor + "and " + (numMotors-1) + " more");
        telemetry.addData("current servo: ", currentservo);
        telemetry.addData("To spin motor: ", "left and right triggers");
        telemetry.addData("To move servo to extremes: ", "left and right bumpers");
        telemetry.addData("To move servo to middle position: ", "a");
        telemetry.addData("To switch motor: ", "up and down on dpad");
        telemetry.addData("To switch servo: ", "left and right on dpad");
        telemetry.addData("To cycle number of motors used: ", "y");
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
            if(button == ControllerInput.Button.Y){ //cycles number of motors affected
                numMotors += 1;
                if(numMotors > 4) numMotors = 1;
            }
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
            if(button == ControllerInput.Button.A) setServoSpeeds(0.5);
            if(button == ControllerInput.Button.B);
            if(button == ControllerInput.Button.X);
            if(button == ControllerInput.Button.Y);
            //Bumpers
            if(button == ControllerInput.Button.LB) setServoSpeeds(1);
            if(button == ControllerInput.Button.RB) setServoSpeeds(0);
            //Triggers
            if(button == ControllerInput.Button.LT) setMotorSpeeds(gamepad1.left_trigger);
            if(button == ControllerInput.Button.RT) setMotorSpeeds(-gamepad1.right_trigger);
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

    private void setServoSpeeds(double speed){
        for (int i = 0; i < numMotors; i++) {
            int servoIndex = i+currentservo;
            if(servoIndex > 5) servoIndex -= 6;
            servos.getServos()[servoIndex].setPosition(speed);
        }
    }

    private void setMotorSpeeds(double speed){
        for (int i = 0; i < numMotors; i++) {
            int motorIndex = i+currentmotor;
            if(motorIndex > 3) motorIndex -= 4;
            motors.getMotors()[motorIndex].setPower(speed);
        }
    }
}
