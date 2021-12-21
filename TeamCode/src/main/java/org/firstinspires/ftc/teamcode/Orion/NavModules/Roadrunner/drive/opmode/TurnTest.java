package org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.StandardMecanumDrive;
import org.firstinspires.ftc.teamcode._RobotCode._Defaults._DefaultRRRobotProfile;
import org.firstinspires.ftc.teamcode._RobotCode._Defaults._DefaultRRTuningProfile;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        StandardMecanumDrive drive = new StandardMecanumDrive(hardwareMap, new _DefaultRRRobotProfile(), new _DefaultRRTuningProfile());

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
