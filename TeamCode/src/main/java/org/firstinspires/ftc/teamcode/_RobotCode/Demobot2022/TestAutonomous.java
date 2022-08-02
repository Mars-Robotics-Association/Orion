package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Navigation.Archive.FreightFrenzy.FreightFrenzyNavigator;

@Autonomous(name = "TestAutonomous", group = "TEST")
public class TestAutonomous extends LinearOpMode
{
    FreightFrenzyNavigator nav;

    @Override
    public void runOpMode() throws InterruptedException {
        nav.GoToFreightLinear();
    }
}
