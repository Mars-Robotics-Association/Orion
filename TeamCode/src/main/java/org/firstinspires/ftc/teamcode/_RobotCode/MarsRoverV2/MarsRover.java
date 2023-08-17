package org.firstinspires.ftc.teamcode._RobotCode.MarsRoverV2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//all dimensions in meters
@TeleOp(name = "MARS ROVER")
@Config
public class MarsRover extends OpMode
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
    public static double FRONT_OFFSET = 0.28575; //front wheel offset from center in meters
    //measured as 11 inches and 11.5 inches, averaged and converted to meters
    public static double BACK_OFFSET = -0.339725; //back wheel offset from center (should be negative) in meters
    //measured as 13.25 inches and 13.5 inches, averaged and converted to meters

    public static double FRONT_AXEL_LENGTH = 0.27305; //distance from center of robot to wheels on axel (x) axis in meters
    //measured as 21.5 inches servo axel to servo axel, divided in half, converted to meters
    public static double MID_AXEL_LENGTH = 0.3175; //distance from center of robot to wheels on axel (x) axis in meters
    //measured as 25 inches middle of wheel to middle of wheel, divided in half, converted to meters
    public static double BACK_AXEL_LENGTH = 0.26035; //distance from center of robot to wheels on axel (x) axis in meters
    //measured as 20.5 inches servo axel to servo axel, divided in half, converted to meters


    public static double MIN_TURN_RADIUS = 0.6; //min turn radius (must be greater than axel length)
    public static double TURN_RADIUS_MULTIPLIER = 0.1; //how gradually to ramp between infinite and minimum radius

    public static double MAX_SPEED = 0.8; //max speed of motors, for now should be no greater than 0.8 to allow spillover speeds

    public static double FR_TRIM = 0.07; //trims for each of the servos in servo units
    public static double FL_TRIM = -0.02;
    public static double BR_TRIM = -0.03;
    public static double BL_TRIM = 0.02;

    @Override
    public void init() {
        //Map motors to hardware map
        motorFrontLeft   = hardwareMap.dcMotor.get("motorFrontLeft"  );
        motorBackLeft    = hardwareMap.dcMotor.get("motorBackLeft"   );
        motorFrontRight  = hardwareMap.dcMotor.get("motorFrontRight" );
        motorBackRight   = hardwareMap.dcMotor.get("motorBackRight"  );
        motorMiddleRight = hardwareMap.dcMotor.get("motorMiddleRight");
        motorMiddleLeft  = hardwareMap.dcMotor.get("motorMiddleLeft" );

        servoFrontLeft  = hardwareMap.servo.get("servoFrontLeft" );
        servoFrontRight = hardwareMap.servo.get("servoFrontRight");
        servoBackLeft   = hardwareMap.servo.get("servoBackLeft"  );
        servoBackRight  = hardwareMap.servo.get("servoBackRight" );


        //sets inital power to 0
        motorFrontRight .setPower(0);
        motorFrontLeft  .setPower(0);
        motorBackRight  .setPower(0);
        motorBackLeft   .setPower(0);
        motorMiddleRight.setPower(0);
        motorMiddleLeft .setPower(0);

        //set ftc dashboard
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        //Get gamepad input
        double turnFactor = gamepad1.right_stick_x;
        double linearSpeed = gamepad1.left_stick_y; //remember max speed is one- cant go further up

        //make sure dead zones exist
        if(Math.abs(turnFactor)<0.05) turnFactor = 0;
        if(Math.abs(linearSpeed)<0.05) linearSpeed = 0;

        //multiplier for turning, LEFT TURN IS POSITIVE, RIGHT IS NEGATIVE
        double turnDirectionMultiplier = Math.copySign(1, turnFactor);

        ////CALCULATE TURN RADIUS (WORKS)

        //uses rational function to ramp down from infinite turn radius to whatever the min is
        double turnRadius = (TURN_RADIUS_MULTIPLIER / Math.abs(turnFactor)) + (MIN_TURN_RADIUS - TURN_RADIUS_MULTIPLIER);

        telemetry.addData("Turn radius", turnRadius);
        telemetry.addData("Turn direction", turnDirectionMultiplier);
        telemetry.addLine();


        //spot turn if "a" button held, else drive normally
        if(gamepad1.dpad_left) spotTurn(0.5);
        else if(gamepad1.dpad_right) spotTurn(-0.5);
        else {
            //Use turn radius to align servos
            alignServos(turnDirectionMultiplier, turnRadius);

            //Use radius and speed to move along path
            setMotorSpeeds(linearSpeed, turnDirectionMultiplier, turnRadius);
        }

        if(gamepad2.x){
            motorFrontRight .setPower(0);
            motorFrontLeft  .setPower(0);
            motorBackRight  .setPower(0);
            motorBackLeft   .setPower(0);
            motorMiddleRight.setPower(0);
            motorMiddleLeft .setPower(0);
            requestOpModeStop();
        }

        telemetry.update();
    }



    private void alignServos(double turnDirectionMultiplier, double turnRadius) {
        ////CALCULATE WHEEL ROTATION ANGLES (WORKS- might need some algorithm fixes, see below)

        //TODO: check and possibly fix algorithms as outer wheels seem to be not rotating enough- inner wheels look fine
        //finds angle AB in triangle ABC where A is turn radius, B is distance from center of robot to front or back, and C is distance from wheel to center of turning circle
        //must multiply axel length by turn direction multiplier depending on which wheels are inner / outer, must multiply buy turn direction multiplier at end as well
        double frontRightRad = Math.atan2(FRONT_OFFSET, (turnRadius + (FRONT_AXEL_LENGTH* turnDirectionMultiplier))) * turnDirectionMultiplier;
        double frontLeftRad  = Math.atan2(FRONT_OFFSET, (turnRadius + (FRONT_AXEL_LENGTH*-turnDirectionMultiplier))) * turnDirectionMultiplier;
        double backRightRad  = Math.atan2(BACK_OFFSET , (turnRadius + (BACK_AXEL_LENGTH * turnDirectionMultiplier))) * turnDirectionMultiplier;
        double backLeftRad   = Math.atan2(BACK_OFFSET , (turnRadius + (BACK_AXEL_LENGTH *-turnDirectionMultiplier))) * turnDirectionMultiplier;

        telemetry.addData("Front right degrees", Math.toDegrees(frontRightRad));
        telemetry.addData("Front left degrees" , Math.toDegrees(frontLeftRad ));
        telemetry.addData("Back right degrees" , Math.toDegrees(backRightRad ));
        telemetry.addData("Back left degrees"  , Math.toDegrees(backLeftRad  ));
        telemetry.addLine();


        ////MOVE SERVOS TO POSITIONS (WORKS)

        //converts radians into servo position using handy map() utility function
        double frontRightServoPos = map(Math.toDegrees(frontRightRad), -150, 150, 0, 1);
        double frontLeftServoPos  = map(Math.toDegrees(frontLeftRad ), -150, 150, 0, 1);
        double backRightServoPos  = map(Math.toDegrees(backRightRad ), -150, 150, 0, 1);
        double backLeftServoPos   = map(Math.toDegrees(backLeftRad  ), -150, 150, 0, 1);

        //if the angle is infinite, point the servos forwards (aka at 0 degrees)
        if(Double.isNaN(frontRightServoPos)) frontRightServoPos = 0.5;
        if(Double.isNaN(frontLeftServoPos )) frontLeftServoPos  = 0.5;
        if(Double.isNaN(backRightServoPos )) backRightServoPos  = 0.5;
        if(Double.isNaN(backLeftServoPos  )) backLeftServoPos   = 0.5;

        //actually move the servos
        servoFrontRight.setPosition(frontRightServoPos + FR_TRIM);
        servoFrontLeft .setPosition(frontLeftServoPos  + FL_TRIM);
        servoBackRight .setPosition(backRightServoPos  + BR_TRIM);
        servoBackLeft  .setPosition(backLeftServoPos   + BL_TRIM);

        telemetry.addData("Front right servo pos", frontRightServoPos);
        telemetry.addData("Front left servo pos" , frontLeftServoPos );
        telemetry.addData("Back right servo pos" , backRightServoPos );
        telemetry.addData("Back left servo pos"  , backLeftServoPos  );
        telemetry.addLine();
    }

    private void setMotorSpeeds(double linearSpeed, double turnDirectionMultiplier, double turnRadius) {
        //// CALCULATE DRIVE SPEEDS (WORKS)

        //starts by finding individual turn radii of all the wheels using below equation
        //sqrt(frontOffset^2 + (axelLength*sideMult)^2)
        double frontRightRadius = distance(FRONT_OFFSET, turnRadius + (FRONT_AXEL_LENGTH * turnDirectionMultiplier));
        double frontLeftRadius  = distance(FRONT_OFFSET, turnRadius + (FRONT_AXEL_LENGTH *-turnDirectionMultiplier));
        double backRightRadius  = distance(BACK_OFFSET , turnRadius + (BACK_AXEL_LENGTH  * turnDirectionMultiplier));
        double backLeftRadius   = distance(BACK_OFFSET , turnRadius + (BACK_AXEL_LENGTH  *-turnDirectionMultiplier));
        double midRightRadius   = turnRadius + (MID_AXEL_LENGTH* turnDirectionMultiplier);
        double midLeftRadius    = turnRadius + (MID_AXEL_LENGTH*-turnDirectionMultiplier);

        telemetry.addData("Front right radius", frontRightRadius);
        telemetry.addData("Front left radius" , frontLeftRadius);
        telemetry.addData("Back right radius" , backRightRadius);
        telemetry.addData("Back left radius"  , backLeftRadius);
        telemetry.addData("Mid right radius"  , midRightRadius);
        telemetry.addData("Mid left radius"   , midLeftRadius);
        telemetry.addLine();

        // Motors midLeft or midRight are already maximums,
        // so we'll Math.max both along with MAX_SPEED
        double motorMaximum = Math.max(midRightRadius, midLeftRadius);
        double maximum = Math.max(MAX_SPEED, motorMaximum * linearSpeed);


        double frontRightSpeed  = (linearSpeed * frontRightRadius) / maximum;
        double frontLeftSpeed   = (linearSpeed * frontLeftRadius ) / maximum;
        double backRightSpeed   = (linearSpeed * backRightRadius ) / maximum;
        double backLeftSpeed    = (linearSpeed * backLeftRadius  ) / maximum;
        double midRightSpeed    = (linearSpeed * midRightRadius  ) / maximum;
        double midLeftSpeed     = (linearSpeed * midLeftRadius   ) / maximum;


        //actually apply speeds- negatives applied here for right vs left side
        motorFrontRight .setPower( frontRightSpeed);
        motorFrontLeft  .setPower(-frontLeftSpeed );
        motorBackRight  .setPower( backRightSpeed );
        motorBackLeft   .setPower(-backLeftSpeed  );
        motorMiddleRight.setPower( midRightSpeed  );
        motorMiddleLeft .setPower(-midLeftSpeed   );

        telemetry.addData("Front right speed", frontRightSpeed);
        telemetry.addData("Front left speed" , frontLeftSpeed );
        telemetry.addData("Back right speed" , backRightSpeed );
        telemetry.addData("Back left speed"  , backLeftSpeed  );
        telemetry.addData("Mid right speed"  , midRightSpeed  );
        telemetry.addData("Mid left speed"   , midLeftSpeed   );
        telemetry.addLine();
    }

    private void spotTurn(double speed){
        ////MOVE SERVOS TO POSITIONS

        //converts radians into servo position using handy map() utility function
        double frontRightServoPos = map( 45, -150, 150, 0, 1);
        double frontLeftServoPos  = map(-45, -150, 150, 0, 1);
        double backRightServoPos  = map(-45, -150, 150, 0, 1);
        double backLeftServoPos   = map( 45, -150, 150, 0, 1);

        //actually move the servos
        servoFrontRight.setPosition(frontRightServoPos + FR_TRIM);
        servoFrontLeft .setPosition(frontLeftServoPos  + FL_TRIM);
        servoBackRight .setPosition(backRightServoPos  + BR_TRIM);
        servoBackLeft  .setPosition(backLeftServoPos   + BL_TRIM);



        telemetry.addData("Front right servo pos", frontRightServoPos);
        telemetry.addData("Front left servo pos" , frontLeftServoPos);
        telemetry.addData("Back right servo pos" , backRightServoPos);
        telemetry.addData("Back left servo pos"  , backLeftServoPos);
        telemetry.addLine();

        ////SET MOTOR SPEEDS
        double frontRightRadius = distance(FRONT_OFFSET, FRONT_AXEL_LENGTH);
        double frontLeftRadius  = distance(FRONT_OFFSET, FRONT_AXEL_LENGTH);
        double backRightRadius  = distance(BACK_OFFSET , BACK_AXEL_LENGTH );
        double backLeftRadius   = distance(BACK_OFFSET , BACK_AXEL_LENGTH );

        double motorMaximum = maxNum(MID_AXEL_LENGTH,
                frontRightRadius,
                frontLeftRadius,
                backRightRadius,
                backLeftRadius);

        double maximum = Math.max(motorMaximum * speed, MAX_SPEED);

        //actually apply speeds- no negatives because we are turning
        motorFrontRight .setPower((frontRightRadius * speed) / maximum);
        motorFrontLeft  .setPower((frontLeftRadius  * speed) / maximum);
        motorBackRight  .setPower((backRightRadius  * speed) / maximum);
        motorBackLeft   .setPower((backLeftRadius   * speed) / maximum);
        motorMiddleRight.setPower((MID_AXEL_LENGTH  * speed) / maximum);
        motorMiddleLeft .setPower((MID_AXEL_LENGTH  * speed) / maximum);
    }

    ////UTILITY////
    private double distance(double y, double x) {
        return Math.sqrt((x * x) + (y * y));
    }
    private double maxNum(double ...arr) {
        double acc = arr[0];
        for (int i = 1; i < arr.length; i++) {
            acc = Math.max(acc, arr[i]);
        }
        return acc;
    }
    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        double slope = 1 * (out_max-out_min) / (in_max - in_min);
        return out_min + slope * (x - in_min);
    }
}
