package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "JUAN FRONT", group = "JUAN")
@Config
public class JuanAutonomousCamera extends LinearOpMode
{
    Juan robot;

    enum ParkPaths{
        A(),
        B(),
        C();


    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan(this,true,true, false);
        robot.init();
        waitForStart();
        robot.start();
        robot.getChassis().setHeadlessMode(true);

        SleeveColor result = robot.payload.getScanner().scan();


    }


}
