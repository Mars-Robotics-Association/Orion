package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

/**
 * Controls the robot using gamepad input.
 */
public class GamepadDriver extends Behavior {

    /**
     * {@inheritDoc}
     */
    @Override
    protected void init() throws Exception {
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
    protected void update() throws Exception {
//        if(gamepad.left_stick_button){ // Lock to an axis
//            if (Math.abs(gamepad.left_stick_x) > Math.abs(gamepad.left_stick_y)){
//                // Lock to spinning
//                drivetrain.spinTurn(gamepad.left_stick_x);
//            }else{
//                // Lock to forward/backward
//                drivetrain.driveBackAndForth(gamepad.left_stick_y);
//            }
//        }else{
//            // Drive normally
//            drivetrain.fullDrive(gamepad.left_stick_x * 24, gamepad.left_stick_y);
//        }

        assert opMode != null;

        Telemetry t = opMode.telemetry;
        RoverDrivetrain drivetrain = getBehavior(RoverDrivetrain.class);
        Gamepad gamepad = opMode.gamepad1;

        assert t != null;
        assert drivetrain != null;
        assert opMode.gamepad1 != null;

        t.addData("Driving at radius (inches) ", gamepad.left_stick_x * 24);
        t.addData("Driving at speed ", gamepad.left_stick_y);

        drivetrain.fullDrive(gamepad.left_stick_x * 24, gamepad.left_stick_y);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void stop() {}
}
