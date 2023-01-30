package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Camera;

@Autonomous(name = "JUAN CAMERA", group = "JUAN")
@Config
public class JuanAutonomousCamera extends LinearOpMode
{
    Juan robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan(this,true,true, false);
        robot.init();

        JuanPayload.SleeveScanner scanner = robot.getPayload().getScanner();

        waitForStart();
        robot.start();
        robot.getChassis().setHeadlessMode(true);

        robot.getChassis().rawDrive(0, -5, 0);
        sleep(500);
        SleeveColor color = scanner.runScan();

        doSomething(color.ordinal() + 1);
    }

    private void doSomething(int count){
        MecanumChassis chassis = robot.getChassis();
        for(int i=0;i<count;i++){
            chassis.rawTurn(1);
            sleep(500);
        }
    }
}
