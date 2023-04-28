package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.HeadController;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain.MotorConfig;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.Safeguards;

import java.io.IOException;

@TeleOp(group = "Mars Rover")
public class MarsRoverTeleop extends OpMode {
    @Override
    public void init(){
        Behavior.systemInit(this,
                new Safeguards(),
                new RoverDrivetrain(
                        new MotorConfig(){
                            double
                        }
                ),
                new HeadController()
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
