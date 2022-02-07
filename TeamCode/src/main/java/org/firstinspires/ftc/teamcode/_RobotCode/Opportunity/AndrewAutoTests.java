package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@Autonomous(name = "Andrew rotation test", group = "All")
public class AndrewAutoTests extends LinearOpMode
{
    AndrewRobot andrew;
    _ChassisProfile chassisProfile;

    @Override
    public void runOpMode() throws InterruptedException {
        andrew = new AndrewRobot(this, true, false,false,this);
        andrew.init();


        waitForStart();
        andrew.StartCoreRobotModules();

        andrew.ResetGyro();
        andrew.GetImu().ResetGyro();

                double startTime = getRuntime();
        while(getRuntime()<startTime+30){
            if(!opModeIsActive()) return;
            telemetry.clear();
            telemetry.addData("distRotation",andrew.getRotationFromDistanceSensors());
            telemetry.update();
        }

    }

    void waitForMotors(AndrewRobot bot, int targetPosition){
        int avgDistance = 0;
        int sum = 0;
        int[] initialPositions = new int[4];
        for(int i = 0; i<4; i++)
            initialPositions[i] = (int)bot.getMotorTicks()[i];

        while (avgDistance<targetPosition) {
            if(!opModeIsActive()) return;
            telemetry.addData("avgDistance",avgDistance);
            telemetry.addData("sum",sum);
            telemetry.addData("targetPosition",targetPosition);
            telemetry.addData("initial[0]",initialPositions[0]);
            telemetry.addData("motor0",(int)bot.getMotorTicks()[0]);
            telemetry.update();

//            synchronized (runningNotifier) {
//                try {
//                    runningNotifier.wait();
//                } catch (InterruptedException e) {
//                    Thread.currentThread().interrupt();
//                    return;
//                }
//            }
            sum = 0;
            for(int i = 0; i<4; i++)
                sum+=Math.abs((int)bot.getMotorTicks()[i]-initialPositions[i]);
            avgDistance = sum/4;
        }
        return;
    }




}
