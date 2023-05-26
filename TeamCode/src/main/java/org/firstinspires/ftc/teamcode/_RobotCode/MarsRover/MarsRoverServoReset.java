package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.Safeguards;

@TeleOp(group = "Mars Rover")
public class MarsRoverServoReset extends OpMode {
    @Override
    public void init(){
        try {
            Behavior.systemInit(this,
                    new Safeguards(),
                    new RoverDrivetrain(MarsDrivetrainConfig.getConfig())
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void start() {
        try {
            Behavior.systemStart();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        try {
            RoverDrivetrain drivetrain = Behavior.getBehavior(RoverDrivetrain.class);
            drivetrain.resetServos();

        } catch (Behavior.BehaviorNotFound e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void loop() {
        try {
            Behavior.systemUpdate();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void stop() {
        try {
            Behavior.systemStop();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
