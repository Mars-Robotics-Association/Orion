package org.firstinspires.ftc.teamcode._RobotCode.MarsRover;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain;
import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors.RoverDrivetrain.DriveUnit;

@Autonomous(group = "Mars Rover")
public class MarsRoverTestingDance extends LinearOpMode {
    private void hold(){
        sleep(1000);
    }

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        double servoExtreme = (5. / 6.) * Math.PI;

        RoverDrivetrain.hardwareMap = this.hardwareMap;

        DriveUnit[] driveUnits = MarsDrivetrainConfig.getConfig();

        waitForStart();

        for (DriveUnit unit: driveUnits) {
            if(unit.hasServo())unit.setRadians(servoExtreme, false);
        }

        hold();

        for (DriveUnit unit: driveUnits) {
            if(unit.hasServo())unit.setRadians(0);
        }

        hold();

        for (DriveUnit unit: driveUnits) {
            if(unit.hasServo())unit.setRadians(servoExtreme, true);
        }

        hold();

        for (DriveUnit unit: driveUnits) {
            if(unit.hasServo())unit.setRadians(0);
        }

        hold();

        for (DriveUnit unit: driveUnits) {
            unit.setSpeed(+.05);
        }

        hold();

        for (DriveUnit unit: driveUnits) {
            unit.setSpeed(0);
        }

        hold();

        for (DriveUnit unit: driveUnits) {
            unit.setSpeed(-.05);
        }

        hold();

        for (DriveUnit unit: driveUnits) {
            unit.setSpeed(0);
        }
    }
}
