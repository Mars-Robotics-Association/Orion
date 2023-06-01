package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.Safeguards;

@TeleOp(group = "Mars Rover")
public class MarsRoverServoReset extends OpMode {
    @Override
    public void init(){
        Behavior.systemInit(this,
                new RoverDrivetrain(MarsDrivetrainConfig.getConfig())
        );
    }

    @Override
    public void start() {
        Behavior.systemStart();

        try {
            RoverDrivetrain drivetrain = Behavior.getBehavior(RoverDrivetrain.class);
            drivetrain.resetServos();
        } catch (Exception e) {
<<<<<<< HEAD
            Behavior.sendException(e, "Start");
=======
            Behavior.sendException(e);
>>>>>>> main
        }
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
