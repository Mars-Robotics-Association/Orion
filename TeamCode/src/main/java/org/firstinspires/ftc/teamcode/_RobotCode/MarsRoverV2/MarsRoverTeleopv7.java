package org.firstinspires.ftc.teamcode._RobotCode.MarsRoverV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mars Roboter v7")
@Config
public class MarsRoverTeleopv7 extends OpMode
{
    FtcDashboard dashboard;

    //Hardware components
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorMiddleRight;
    DcMotor motorMiddleLeft;

    Servo servoFrontLeft;
    Servo servoFrontRight;
    Servo servoBackLeft;
    Servo servoBackRight;

    //Config variables
    public static double FRONT_OFFSET = 10;
    public static double BACK_OFFSET = -12.5;
    public static double AXEL_LENGTH = 10;

    public static double MIN_TURN_RADIUS = 12;
    public static double TURN_RADIUS_MULTIPLIER = 5;

    public static double MAX_SPEED = 0.5;

    @Override
    public void init() {
        //Map motors
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorMiddleRight = hardwareMap.dcMotor.get("motorMiddleRight");
        motorMiddleLeft = hardwareMap.dcMotor.get("motorMiddleLeft");

        servoFrontLeft = hardwareMap.servo.get("servoFrontLeft");
        servoFrontRight = hardwareMap.servo.get("servoFrontRight");
        servoBackLeft = hardwareMap.servo.get("servoBackLeft");
        servoBackRight = hardwareMap.servo.get("servoBackRight");

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        //Get controller input
        double turnFactor = gamepad1.right_stick_x;
        double linearSpeed = gamepad1.left_stick_y; //remember max speed is one- cant go further up
        //make sure dead zones exist
        //if(Math.abs(turnFactor)<0.1) turnFactor = 0;
        if(Math.abs(linearSpeed)<0.1) linearSpeed = 0;

        //multiplier for turning, left turn is positive
        double turnDirectionMultiplier = turnFactor/Math.abs(turnFactor);

        //calculate turn radius (WORKS)
        //uses rational function to ramp down from infinite turn radius to whatever the min is
        double turnRadius = (Math.pow(Math.abs(turnFactor),-1) * TURN_RADIUS_MULTIPLIER) + (MIN_TURN_RADIUS - TURN_RADIUS_MULTIPLIER);
        telemetry.addData("Turn radius", turnRadius);
        telemetry.addData("Turn direction", turnDirectionMultiplier);
        telemetry.addLine();

        //calculate servo angles (WORKS)
        double frontRightRad = Math.atan(FRONT_OFFSET/(turnRadius+(AXEL_LENGTH*(turnDirectionMultiplier)))) * turnDirectionMultiplier;
        double frontLeftRad = Math.atan(FRONT_OFFSET/(turnRadius+(AXEL_LENGTH*(-turnDirectionMultiplier)))) * turnDirectionMultiplier;
        double backRightRad = Math.atan(BACK_OFFSET/(turnRadius+(AXEL_LENGTH*(turnDirectionMultiplier)))) * turnDirectionMultiplier;
        double backLeftRad = Math.atan(BACK_OFFSET/(turnRadius+(AXEL_LENGTH*(-turnDirectionMultiplier)))) * turnDirectionMultiplier;

        telemetry.addData("Front right degrees", Math.toDegrees(frontRightRad));
        telemetry.addData("Front left degrees", Math.toDegrees(frontLeftRad));
        telemetry.addData("Back right degrees", Math.toDegrees(backRightRad));
        telemetry.addData("Back left degrees", Math.toDegrees(backLeftRad));
        telemetry.addLine();

        //move servos to angles
        double frontRightServoPos = map(Math.toDegrees(frontRightRad), -150, 150, 0, 1);
        double frontLeftServoPos = map(Math.toDegrees(frontLeftRad), -150, 150, 0, 1);
        double backRightServoPos = map(Math.toDegrees(backRightRad), -150, 150, 0, 1);
        double backLeftServoPos = map(Math.toDegrees(backLeftRad), -150, 150, 0, 1);

        if(Double.isNaN(frontRightServoPos)) frontRightServoPos = 0.5;
        if(Double.isNaN(frontLeftServoPos)) frontLeftServoPos = 0.5;
        if(Double.isNaN(backRightServoPos)) backRightServoPos = 0.5;
        if(Double.isNaN(backLeftServoPos)) backLeftServoPos = 0.5;

        servoFrontRight.setPosition(frontRightServoPos);
        servoFrontLeft.setPosition(frontLeftServoPos);
        servoBackRight.setPosition(backRightServoPos);
        servoBackLeft.setPosition(backLeftServoPos);

        telemetry.addData("Front right servo pos", frontRightServoPos);
        telemetry.addData("Front left servo pos", frontLeftServoPos);
        telemetry.addData("Back right servo pos", backRightServoPos);
        telemetry.addData("Back left servo pos", backLeftServoPos);
        telemetry.addLine();

        //calculate drive speeds
        //sqrt(frontOffset^2 + (axelLength*sideMult)^2)
        double frontRightRadius = Math.sqrt(Math.pow(FRONT_OFFSET, 2) + Math.pow(turnRadius+(AXEL_LENGTH*(turnDirectionMultiplier)), 2));
        double frontLeftRadius = Math.sqrt(Math.pow(FRONT_OFFSET, 2) + Math.pow(turnRadius+(AXEL_LENGTH*(-turnDirectionMultiplier)), 2));
        double backRightRadius = Math.sqrt(Math.pow(BACK_OFFSET, 2) + Math.pow(turnRadius+(AXEL_LENGTH*(turnDirectionMultiplier)), 2));
        double backLeftRadius = Math.sqrt(Math.pow(BACK_OFFSET, 2) + Math.pow(turnRadius+(AXEL_LENGTH*(-turnDirectionMultiplier)), 2));
        double midRightRadius = turnRadius+(AXEL_LENGTH*(turnDirectionMultiplier));
        double midLeftRadius = turnRadius+(AXEL_LENGTH*(-turnDirectionMultiplier));

        telemetry.addData("Front right radius", frontRightRadius);
        telemetry.addData("Front left radius", frontLeftRadius);
        telemetry.addData("Back right radius", backRightRadius);
        telemetry.addData("Back left radius", backLeftRadius);
        telemetry.addData("Mid right radius", midRightRadius);
        telemetry.addData("Mid left radius", midLeftRadius);
        telemetry.addLine();

        double frontRightSpeed = 0;
        double frontLeftSpeed = 0;
        double backRightSpeed = 0;
        double backLeftSpeed = 0;
        double midRightSpeed = 0;
        double midLeftSpeed = 0;

        //if turning left, use right wheel as reference for speed ratios
        if(turnDirectionMultiplier > 0){
            frontRightSpeed = MAX_SPEED * linearSpeed * (frontRightRadius/midRightRadius);
            frontLeftSpeed = MAX_SPEED * linearSpeed * (frontLeftRadius/midRightRadius);
            backRightSpeed = MAX_SPEED * linearSpeed * (backRightRadius/midRightRadius);
            backLeftSpeed = MAX_SPEED * linearSpeed * (backLeftRadius/midRightRadius);
            midRightSpeed = MAX_SPEED * linearSpeed * (midRightRadius/midRightRadius);
            midLeftSpeed = MAX_SPEED * linearSpeed * (midLeftRadius/midRightRadius);
        }
        //if turning right, use left wheel as reference for speed ratios
        else{
            frontRightSpeed = MAX_SPEED * linearSpeed * (frontRightRadius/midLeftRadius);
            frontLeftSpeed = MAX_SPEED * linearSpeed * (frontLeftRadius/midLeftRadius);
            backRightSpeed = MAX_SPEED * linearSpeed * (backRightRadius/midLeftRadius);
            backLeftSpeed = MAX_SPEED * linearSpeed * (backLeftRadius/midLeftRadius);
            midRightSpeed = MAX_SPEED * linearSpeed * (midRightRadius/midLeftRadius);
            midLeftSpeed = MAX_SPEED * linearSpeed * (midLeftRadius/midLeftRadius);
        }

        if(turnFactor == 0){
            frontRightSpeed = MAX_SPEED * linearSpeed;
            frontLeftSpeed = MAX_SPEED * linearSpeed;
            backRightSpeed = MAX_SPEED * linearSpeed;
            backLeftSpeed = MAX_SPEED * linearSpeed;
            midRightSpeed = MAX_SPEED * linearSpeed;
            midLeftSpeed = MAX_SPEED * linearSpeed;
        }

        motorFrontRight.setPower(frontRightSpeed);
        motorFrontLeft.setPower(-frontLeftSpeed);
        motorBackRight.setPower(backRightSpeed);
        motorBackLeft.setPower(-backLeftSpeed);
        motorMiddleRight.setPower(midRightSpeed);
        motorMiddleLeft.setPower(-midLeftSpeed);

        telemetry.addData("Front right speed", frontRightSpeed);
        telemetry.addData("Front left speed", frontLeftSpeed);
        telemetry.addData("Back right speed", backRightSpeed);
        telemetry.addData("Back left speed", backLeftSpeed);
        telemetry.addData("Mid right speed", midRightSpeed);
        telemetry.addData("Mid left speed", midLeftSpeed);
        telemetry.addLine();



        telemetry.update();
    }


    ////UTILITY////
    private double distance(double y, double x) {
        return Math.sqrt((x * x) + (y * y));
    }
    private double maxNum(double[] arr) {
        double acc = arr[0];
        for (int i = 1; i < arr.length; i++) {
            acc = Math.max(acc, arr[i]);
        }
        return acc;
    }
    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        //return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        double slope = 1 * (out_max-out_min) / (in_max - in_min);
        return out_min + slope * (x - in_min);
    }
}
