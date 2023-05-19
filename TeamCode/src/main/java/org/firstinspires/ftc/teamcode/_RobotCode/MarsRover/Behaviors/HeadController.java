package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

/**
 * Controls the servos
 */
public class HeadController extends Behavior {
    // Pitch = up/down
    // Yaw = left/right

    private Servo pitchServo;
    private Servo yawServo;

    // just getter and setter functions. Nothing much here.
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
        // Use the hardware map to get hardware references.
        // STOPSHIP: 5/7/2023 These servo names are based on vague assumptions.
        yawServo = hardwareMap.servo.get("S7");
        pitchServo = hardwareMap.servo.get("S8");

        // Check for null references.
        if(yawServo == null || pitchServo == null)throw new Exception("Check head servos!");
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
