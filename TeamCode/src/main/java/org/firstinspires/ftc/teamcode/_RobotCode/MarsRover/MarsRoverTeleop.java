package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.GamepadDriver;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.MinimalCameraHead;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain;

@TeleOp(group = "Mars Rover")
public class MarsRoverTeleop extends OpMode {
    @Override
    public void init() {
        Behavior.systemInit(this,
                new RoverDrivetrain(new RoverDrivetrain.Configuration(){
                    /**Distance from any corner wheel to the Z axis*/
                    private final double cornerBaseHalf = 1;
                    /**Distance from either middle wheel to the Z axis*/
                    private final double centerBaseHalf = 1;
                    /**Distance from the center axle to front or back axle*/
                    private final double ZMiddleDistance = 1;
                }),
                new MinimalCameraHead(),
                new GamepadDriver()
        );
    }

    @Override
    public void start() {
        Behavior.systemStart();
    }

    @Override
    public void loop() {
        Behavior.systemUpdate();
    }

    @Override
    public void stop() {
        Behavior.systemStop();
    }
}
