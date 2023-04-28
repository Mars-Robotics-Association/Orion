package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

public class HeadController extends Behavior {
    // Pitch = up/down
    // Yaw = left/right

    private Servo pitchServo;
    private Servo yawServo;

    public double getPitch(){
        return pitchServo.getPosition();
    }

    public double getYaw(){
        return pitchServo.getPosition();
    }

    public void setPitch(double pos){
        pitchServo.setPosition(pos);
    }

    public void setYaw(double pos){
        yawServo.setPosition(pos);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void init() throws Exception {
        yawServo = hardwareMap.servo.get("S4");
        pitchServo = hardwareMap.servo.get("S5");

        if(yawServo == null || pitchServo == null)throw new Exception("Check head servos!");

        setPitch(0);
        setYaw(0);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void start() {

    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void update() {

    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void stop() {

    }
}
