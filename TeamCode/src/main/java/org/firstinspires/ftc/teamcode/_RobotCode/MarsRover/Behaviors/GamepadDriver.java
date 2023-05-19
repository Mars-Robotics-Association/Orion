package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

/**
 * Controls the robot using gamepad input.
 */
public class GamepadDriver extends Behavior {

    // References

    private Gamepad gamepad;
    private RoverDrivetrain drivetrain;
    private HeadController headController;

    // Tweaking Vars

    /** Speed in pos/s the pitch (up/down) servo will rotate */
    final double pitchSpeed = 0.01;
    /** Speed in pos/s the yaw (left/right) servo will rotate */
    final double yawSpeed = 0.01;
    /** Time since last update in nanoseconds */

    long lastNano = 0;

    /**
     * {@inheritDoc}
     */
    @Override
    protected void init() throws Exception {
        // Calls to getBehavior should be reduced as much as possible.
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
        // Calculate a time delta
        long deltaNano = System.nanoTime() - lastNano;
        double deltaTime = (double)(deltaNano) / 1e-9;

        double pitch = headController.getPitch();
        double yaw = headController.getYaw();

        // Same as headController.pitch += pitchSpeed * gamepad.right_stick_y * deltaTime
        headController.setPitch(pitch + (pitchSpeed*gamepad.right_stick_y*deltaTime));
        // Same as headController.yaw += yawSpeed * gamepad.right_stick_x * deltaTime
        headController.setYaw(yaw + (yawSpeed*gamepad.right_stick_x*deltaTime));

        if(gamepad.left_stick_button){ // Lock to an axis
            if (Math.abs(gamepad.left_stick_x) > Math.abs(gamepad.left_stick_y)){
                // Lock to spinning
                drivetrain.spinTurn(gamepad.left_stick_x);
            }else{
                // Lock to forward/backward
                drivetrain.driveBackAndForth(gamepad.left_stick_y);
            }
        }else{
            // Drive normally
            double turnRadius = gamepad.left_stick_x * 5 + Math.copySign(drivetrain.getMinimumTurnRadius(), gamepad.left_stick_x);
            drivetrain.fullDrive(turnRadius, gamepad.left_stick_y);
        }

        lastNano = System.nanoTime();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void stop() {}
}
