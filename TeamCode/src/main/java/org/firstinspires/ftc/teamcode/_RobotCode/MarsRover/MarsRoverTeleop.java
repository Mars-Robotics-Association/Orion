package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.GamepadDriver;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.Safeguards;

@TeleOp(group = "Mars Rover")
public class MarsRoverTeleop extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Behavior.systemInit(this,
                    new GamepadDriver(),
                    new RoverDrivetrain(MarsDrivetrainConfig.getConfig())
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        waitForStart();

        try {
            Behavior.systemStart();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        while (true){
            try {
                Behavior.systemUpdate();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }
}
