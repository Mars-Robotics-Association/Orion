package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.GamepadDriver;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.Safeguards;

@TeleOp(group = "Mars Rover")
public class MarsRoverTeleop extends OpMode {
    @Override
    public void init(){
        Behavior.systemInit(this,
                new Safeguards(),
                new RoverDrivetrain(MarsDrivetrainConfig.getConfig()),
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
