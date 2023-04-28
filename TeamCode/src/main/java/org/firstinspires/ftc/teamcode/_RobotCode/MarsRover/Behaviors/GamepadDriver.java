package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

/**
 * Safety features intended to keep the robot from falling apart.
 */
public class GamepadDriver extends Behavior {

    // References

    private Gamepad gamepad;
    private RoverDrivetrain drivetrain;
    private HeadController headController;

    // Tweaking Vars

    /** Speed in deg/s the pitch (up/down) servo will rotate */
    final double pitchSpeed = 1;
    /** Speed in deg/s the yaw (left/right) servo will rotate */
    final double yawSpeed = 1;
    /** Time since last update in nanoseconds */



    long lastNano = 0;

    /**
     * {@inheritDoc}
     */
    @Override
    protected void init() {
        drivetrain = getBehavior(RoverDrivetrain.class);
        headController = getBehavior(HeadController.class);
        gamepad = opMode.gamepad1;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void start() {
        lastNano = System.nanoTime();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void update() {
        long deltaNano = System.nanoTime() - lastNano;
        double deltaTime = (double)(deltaNano) / 1e-9;

        double pitch = headController.getPitch();
        double yaw = headController.getYaw();

        headController.setPitch(pitch + (pitchSpeed*gamepad.right_stick_y*deltaTime));
        headController.setYaw(yaw + (yawSpeed*gamepad.right_stick_x*deltaTime));

        if(gamepad.a){
            drivetrain.spinTurn(gamepad.left_stick_x);
        } else if (gamepad.b) {
            drivetrain.driveBackAndForth(gamepad.left_stick_y);
        }else{
            drivetrain.fullDrive(0, gamepad.left_stick_y, null, null);
        }

        lastNano = System.nanoTime();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void stop() {
    }
}
