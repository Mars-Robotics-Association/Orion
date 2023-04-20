package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

public class GamepadDriver extends Behavior {

    private Gamepad gamepad;
    private RoverDrivetrain drivetrain;
    private MinimalCameraHead minimalCameraHead;

    final double pitchSpeed = 1;
    final double yawSpeed = 1;

    long lastNano;

    @Override
    protected void init() {
        drivetrain = getBehavior(RoverDrivetrain.class);
        minimalCameraHead = getBehavior(MinimalCameraHead.class);
        gamepad = opMode.gamepad1;
    }

    @Override
    protected void start() {
        lastNano = System.nanoTime();
    }

    @Override
    protected void update() {
        long deltaNano = System.nanoTime() - lastNano;
        double deltaTime = (double)(deltaNano) / 1e-9;

        double pitch = minimalCameraHead.getPitch();
        double yaw = minimalCameraHead.getYaw();

        minimalCameraHead.setPitch(pitch + (pitchSpeed*gamepad.right_stick_y*deltaTime));
        minimalCameraHead.setYaw(yaw + (yawSpeed*gamepad.right_stick_x*deltaTime));
        lastNano = System.nanoTime();
    }

    @Override
    protected void stop() {
    }
}
