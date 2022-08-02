package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sketchpad
{
    DcMotor motor;
    Servo servo;
    OpMode opMode;

    public void motorsExample(){
        //Initialize motor
        motor = opMode.hardwareMap.dcMotor.get("name");

        //Initialize servo
        servo = opMode.hardwareMap.servo.get("name");

        //Setting servo position or speed
        double servo_pos = 0; //number between 0 and 1
        servo.setPosition(servo_pos);
        //if the servo is in servo mode, 0 goes to min and 1 goes to max
        //if it is in continuous mode, 0 goes at speed -1, 1 at speed 1, and 0.5 stops it

        //Setting motor speed
        double power = 0; //number between -1 and 1
        motor.setPower(power); // -1 goes reverse, 0 stops, and 1 goes forwards

        //Setting motor target position
        int position = 0; //any integer
        motor.setTargetPosition(position); //comes into effect when using "Run_To_Position" mode

        //Setting motor modes
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //does not use encoder to correct speed
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //runs using the encoder to correct speed
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //goes to target position using native PIDs
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //stops motors and sets encoder to zero

    }

    DistanceSensor distanceSensor;
    ColorSensor colorSensor;
    TouchSensor touchSensor;

    public void sensorsExample(){
        //Initialization
        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "name");
        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "name");
        touchSensor = opMode.hardwareMap.get(TouchSensor.class, "name");

        //Using a distance sensor
        double distance = distanceSensor.getDistance(DistanceUnit.CM); //returns distance in cm
        double targetDist = 10;
        if(distance<targetDist) DoSomething(); //if within range, execute given function

        //Using a color sensor
        int intensity = colorSensor.alpha(); //returns how intense the reflected light is
        int red = colorSensor.red(); //returns red channel as an integer
        if(intensity>150) DoSomething(); //if color is light enough, its over a white spot or line

        //Using a touch sensor
        boolean isTouched = touchSensor.isPressed(); //returns a boolean whether sensor is pressed
        if(isTouched) DoSomething(); //if sensor is pressed, do something

        //Make use of telemetry!
        opMode.telemetry.addData("distance: ", distanceSensor.getDistance(DistanceUnit.CM));
        opMode.telemetry.addData("color: ", colorSensor.alpha());
        opMode.telemetry.addData("touch: ", touchSensor.isPressed());

    }

    void DoSomething(){
        opMode.telemetry.addLine("Hi!");
    }
}
