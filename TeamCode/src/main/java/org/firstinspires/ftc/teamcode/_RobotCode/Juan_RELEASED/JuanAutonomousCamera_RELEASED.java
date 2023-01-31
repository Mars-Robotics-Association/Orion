package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

@Autonomous(name = "JUAN CAMERA - RELEASED", group = "JUAN")
@Config
public class JuanAutonomousCamera_RELEASED extends LinearOpMode
{
    Juan_RELEASED robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Juan_RELEASED(this,true,true, false);
        robot.init();

        JuanPayload_RELEASED.SleeveScanner scanner = robot.getPayload().getScanner();

        waitForStart();
        robot.start();
        robot.getChassis().setHeadlessMode(true);

        robot.getChassis().rawDrive(0, -5, 0);
        sleep(500);
        SleeveColor_RELEASED color = scanner.runScan().color;

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
