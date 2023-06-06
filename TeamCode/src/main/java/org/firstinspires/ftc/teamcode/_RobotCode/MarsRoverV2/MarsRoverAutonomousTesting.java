package org.firstinspires.ftc.teamcode._RobotCode.MarsRoverV2;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//The name in quotation marks will appear on the driver station screen
@Autonomous(name = "MarsRoverAutonomousTesting", group = "Competition")

public class MarsRoverAutonomousTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare the motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor motorMiddleRight = hardwareMap.dcMotor.get("motorMiddleRight");
        DcMotor motorMiddleLeft = hardwareMap.dcMotor.get("motorMiddleLeft");

        //Declare the servos
        Servo servoTop = hardwareMap.servo.get("servoTop");
        Servo servoBottom = hardwareMap.servo.get("servoBottom");

        Servo servoFrontLeft = hardwareMap.servo.get("servoFrontLeft");
        Servo servoFrontRight = hardwareMap.servo.get("servoFrontRight");
        Servo servoBackLeft = hardwareMap.servo.get("servoBackLeft");
        Servo servoBackRight = hardwareMap.servo.get("servoBackRight");


        //Declare refrence constants

        double motorSpeed = 0.5;

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        // motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            //double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            // double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            // double rx = gamepad1.right_stick_x;
            /* Denominator is the largest motor power (absolute value) or 1
            This ensures all the powers maintain the same ratio */




            if(true){
                //test motor and encoder
                telemetry.addLine("Testing Motors");
                telemetry.update();

                motorFrontRight.setPower(motorSpeed);
                motorMiddleRight.setPower(motorSpeed);
                motorBackRight.setPower(motorSpeed);

                motorFrontLeft.setPower(-motorSpeed);
                motorMiddleLeft.setPower(-motorSpeed);
                motorBackLeft.setPower(-motorSpeed);

                sleep(5000);

                motorFrontRight.setPower(0);
                motorMiddleRight.setPower(0);
                motorBackRight.setPower(0);

                motorFrontLeft.setPower(0);
                motorMiddleLeft.setPower(0);
                motorBackLeft.setPower(0);

                telemetry.addLine("Motor test complete, encoder values below:");
                telemetry.addData("FrontLeft:", motorFrontLeft.getCurrentPosition());
                telemetry.addData("MiddleLeft:", motorMiddleLeft.getCurrentPosition());
                telemetry.addData("BackLeft:", motorBackLeft.getCurrentPosition());

                telemetry.addData("FrontRight:", motorFrontRight.getCurrentPosition());
                telemetry.addData("MiddleRight:", motorMiddleRight.getCurrentPosition());
                telemetry.addData("BackRight:", motorBackRight.getCurrentPosition());

                if(motorFrontLeft.getCurrentPosition()==0){
                    telemetry.addLine("FRONTLEFT ENCODER FAILURE");
                }
                if(motorMiddleLeft.getCurrentPosition()==0){
                    telemetry.addLine("MIDDLELEFT ENCODER FAILURE");
                }
                if(motorBackLeft.getCurrentPosition()==0){
                    telemetry.addLine("BACKLEFT ENCODER FAILURE");
                }
                if(motorFrontRight.getCurrentPosition()==0){
                    telemetry.addLine("FRONTRIGHT ENCODER FAILURE");
                }
                if(motorMiddleRight.getCurrentPosition()==0){
                    telemetry.addLine("MIDDLERIGHT ENCODER FAILURE");
                }
                if(motorBackRight.getCurrentPosition()==0){
                    telemetry.addLine("BACKRIGHT ENCODER FAILURE");
                }


                telemetry.addLine("Servo Test Beginning");

                telemetry.update();

                sleep(2000);

                servoFrontLeft.setPosition(0.5);
                servoFrontRight.setPosition(0.5);
                servoBackLeft.setPosition(0.5);
                servoBackRight.setPosition(0.5);

                sleep(1000);

                servoFrontLeft.setPosition(0.3);
                servoFrontRight.setPosition(0.3);
                servoBackLeft.setPosition(0.3);
                servoBackRight.setPosition(0.3);

                sleep(1000);

                servoFrontLeft.setPosition(0.5);
                servoFrontRight.setPosition(0.5);
                servoBackLeft.setPosition(0.5);
                servoBackRight.setPosition(0.5);

                sleep(1000);

                servoFrontLeft.setPosition(0.7);
                servoFrontRight.setPosition(0.7);
                servoBackLeft.setPosition(0.7);
                servoBackRight.setPosition(0.7);

                sleep(1000);

                servoFrontLeft.setPosition(0.5);
                servoFrontRight.setPosition(0.5);
                servoBackLeft.setPosition(0.5);
                servoBackRight.setPosition(0.5);

                sleep(1000);

                telemetry.addLine("Servo Test Conclueded");
                telemetry.update();
            }
            requestOpModeStop();

        }
    }
}

