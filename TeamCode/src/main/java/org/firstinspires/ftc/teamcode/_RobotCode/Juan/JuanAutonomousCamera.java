package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "JUAN FRONT CAMERA", group = "JUAN")

@Config
public class JuanAutonomousCamera extends LinearOpMode
{
    Juan robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan(this,true,true, false);
        robot.init();
        waitForStart();
        robot.start();
        robot.getChassis().setHeadlessMode(true);
        JuanNavigation nav = robot.navigator;
        nav.InitializeNavigator(this, robot);
        nav.setMeasuredPose(0, 0, 0);

        SleeveColor result = robot.payload.getScanner().scan();

        switch(result){
            case GREEN:
                nav.goTowardsPose(0, 18, 0, 1);
                nav.goTowardsPose(-18, 18, 0, 1);
                break;
            case ORANGE:
                nav.goTowardsPose(0, 18, 0, 1);
                break;
            case PURPLE:
                nav.goTowardsPose(0, 18, 0, 1);
                nav.goTowardsPose(18, 18, 0, 1);
        }
    }
}
