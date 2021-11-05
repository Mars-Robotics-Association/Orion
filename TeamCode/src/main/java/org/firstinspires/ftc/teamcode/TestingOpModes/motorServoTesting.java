package org.firstinspires.ftc.teamcode.TestingOpModes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "motorServoTest")
public class motorServoTesting extends OpMode{

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
    }

    @Override
    public void loop(){
        //Spin motor forwards/backwards using left trigger
        if (gamepad1.left_trigger > 0.1){
            switch(currentmotor){
                case(1):
                    motor1.setPower(1);
                    break;
                case(2):
                    motor2.setPower(1);
                    break;
                case(3):
                    motor3.setPower(1);
                    break;
                case(4):
                    motor4.setPower(1);
                    break;
            }
        }else if (gamepad1.right_trigger > 0.1){
            switch(currentmotor) {
                case (1):
                    motor1.setPower(-1);
                    break;
                case (2):
                    motor2.setPower(-1);
                    break;
                case (3):
                    motor3.setPower(-1);
                    break;
                case (4):
                    motor4.setPower(-1);
                    break;
            }
        }else{
            switch(currentmotor) {
                case (1):
                    motor1.setPower(0);
                    break;
                case (2):
                    motor2.setPower(0);
                    break;
                case (3):
                    motor3.setPower(0);
                    break;
                case (4):
                    motor4.setPower(0);
                    break;
            }
        }


        //Spin motor 1 forwards/backwards using left trigger
        if (gamepad1.right_bumper){
            switch(currentservo) {
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
                case(5):
                    servo5.setPosition(0);
                    break;
                case(6):
                    servo6.setPosition(0);
                    break;
            }
        }else if (gamepad1.left_bumper){
            switch(currentservo) {
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
                case(5):
                    servo5.setPosition(1);
                    break;
                case(6):
                    servo6.setPosition(1);
                    break;
            }
        }else if(gamepad1.a){
            switch(currentservo) {
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
                case(5):
                    servo5.setPosition(0.5);
                    break;
                case(6):
                    servo6.setPosition(0.5);
                    break;
            }
        }
        telemetry.addData("current motor: ",currentmotor);
        telemetry.addData("current servo: ",currentservo);
        telemetry.addData("To spin motor: ","left and right triggers");
        telemetry.addData("To move servo to extremes: ","left and right bumpers");
        telemetry.addData("To move servo to middle position: ","a");
        telemetry.addData("Make sure you are using ","gamepad 1");
        telemetry.update();
//
//
//        //Spin motor 1 forwards/backwards using left trigger
//        if (gamepad1.dpad_up){
//            motor3.setPower(1);
//        }else if (gamepad1.dpad_down){
//            motor3.setPower(-1);
//        }else{
//            motor3.setPower(0);
//        }
//
//
//        //Spin motor 1 forwards/backwards using left trigger
//        if (gamepad1.x){
//            motor4.setPower(1);
//        }else if (gamepad1.y){
//            motor4.setPower(-1);
//        }else{
//            motor4.setPower(0);
//        }
//
//
//
//
//
//        //Send servo 1 to extremes using left trigger
//        if (gamepad2.left_trigger > 0.1){
//            servo1.setPosition(0);
//        }else if (gamepad2.right_trigger> 0.1){
//            servo1.setPosition(1);
//        }
//
//
//        if (gamepad2.left_bumper){
//            servo2.setPosition(0);
//        }else if (gamepad2.right_bumper){
//            servo2.setPosition(1);
//        }
//
//
//        if (gamepad2.dpad_up){
//            servo3.setPosition(0);
//        }else if (gamepad2.dpad_down) {
//            servo3.setPosition(1);
//        }
//
//
//        if (gamepad2.dpad_left){
//            servo4.setPosition(0);
//        }else if (gamepad2.dpad_right) {
//            servo4.setPosition(1);
//        }
//
//
//        if (gamepad2.x){
//            servo5.setPosition(0);
//        }else if (gamepad2.y) {
//            servo5.setPosition(1);
//        }
//
//
//        if (gamepad2.a){
//            servo6.setPosition(0);
//        }else if (gamepad2.b) {
//            servo6.setPosition(1);
//        }

    }


}
