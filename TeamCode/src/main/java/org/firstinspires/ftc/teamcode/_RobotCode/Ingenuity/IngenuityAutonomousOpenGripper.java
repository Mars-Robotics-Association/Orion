package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Ingenuity Open Wide", group = "Ingenuity")
@Config
public class IngenuityAutonomousOpenGripper extends LinearOpMode {

    IngenuityPowerPlayBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new IngenuityPowerPlayBot(this, true, true, true);
        robot.init();
        robot.getChassis().setHeadlessMode(true);

        waitForStart();
        robot.start();

        robot.gripperOpenWide();
        sleep(1000);

        robot.stop();
    }
}