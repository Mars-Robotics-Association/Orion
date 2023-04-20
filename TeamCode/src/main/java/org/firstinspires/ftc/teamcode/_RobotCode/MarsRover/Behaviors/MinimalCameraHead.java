package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;


public class MinimalCameraHead extends Behavior {
    // Pitch = up/down
    // Yaw = left/right

    private Servo pitchServo;
    private Servo yawServo;

    public double getPitch(){
        return (pitchServo.getPosition() * 270) - 135;
    }

    public double getYaw(){
        return (pitchServo.getPosition() * 270) - 135;
    }

    public void setPitch(double degrees){
        pitchServo.setPosition((degrees / 270) + .5);
    }

    public void setYaw(double degrees){
        yawServo.setPosition((degrees / 270) + .5);
    }

    @Override
    protected void init() {
        yawServo = hardwareMap.servo.get("S0");
        pitchServo = hardwareMap.servo.get("S1");

        setPitch(0);
        setYaw(0);
    }

    @Override
    protected void start() {

    }

    @Override
    protected void update() {

    }

    @Override
    protected void stop() {

    }
}
