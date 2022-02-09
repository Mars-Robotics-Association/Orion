package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@Autonomous(name = "Andrew Auto AW", group = "All")
public class AndrewAutonomous_AW extends LinearOpMode
{
    AndrewRobot andrew;
    _ChassisProfile chassisProfile;

    @Override
    public void runOpMode() throws InterruptedException {
        andrew = new AndrewRobot(this, true, false,false,this);
        andrew.init();

        andrew.GetImu().Start();
        sleep (500);
        double ld_gyro_init = andrew.GetImu().GetRobotAngle();
        andrew.GetImu().ResetGyro();
//        andrew.StartCoreRobotModules();

        waitForStart();
        int armStartPos = andrew.armPos.getCurrentPosition();
        int turntableStartPos = andrew.turntable.getCurrentPosition();
        //andrew.GetImu().ResetGyro();

        int li_iterations = 0;
        double ld_gyro_reset = andrew.GetImu().GetRobotAngle();
        double[] ld_gyro_now = new double[25];
        double ld_error = 0;

        while(li_iterations<25) {
            telemetry.addData("Version:         ", "1.1");
        //    telemetry.addData("Iterations:      ", li_iterations);
            telemetry.addData("IMU Angle Start: ", ld_gyro_init);
            telemetry.addData("IMU Angle Reset: ", ld_gyro_reset);
            //andrew.GetImu().ResetGyro();
            //telemetry.addData("andrewIMU: ", andrew.GetImu().getRotation());
            //telemetry.addData("IMU Angle Pre:   ","");
            ld_gyro_now[li_iterations]=andrew.GetImu().GetRobotAngle();
            for(int i=0;i<li_iterations;i++) {
                telemetry.addData("", ld_gyro_now[i]);
            }
            //telemetry.addData("IMU Angle Pre:   ", andrew.GetImu().GetRobotAngle());
            sleep (1000);

            // Turn to Angle
            // Turn to +45
            //if(li_iterations==0){
            //    ld_error=andrew.TurnToAngle(-45,0.5,1);
            //}

            //telemetry.addData("ld_Error:   ", ld_error);
            //telemetry.addData("IMU Angle Post:   ", andrew.GetImu().GetRobotAngle());
            telemetry.update();

            li_iterations++;
        }

        sleep (25000);

/*
        andrew.RawDrive(0,0.7,0);
        waitForMotors(andrew,650,15); //initial forward
        andrew.Stop();
        sleep(200);

        andrew.RawDrive(90,0.8,0);
        waitForMotors(andrew,300,15); //left to red line
        andrew.Stop();
        sleep(200);

        andrew.RawDrive(0,0.6,0);
        waitForMotors(andrew,100,15); // forward to wall
        andrew.Stop();
        sleep(200);

        andrew.duckyMotor.setPower(-0.8);

        andrew.RawDrive(270,0.4,0);
        waitForMotors(andrew,370,5); //right into turntable
        andrew.Stop();
        sleep(1500);
        andrew.duckyMotor.setPower(0);
        andrew.gripper.setPower(1);

        andrew.RawDrive(90,0.6,0);
        waitForMotors(andrew,500,15);
        andrew.Stop();
        sleep(150);

        andrew.RawDrive(0,0.5,0);
        waitForMotors(andrew,220,3); // align with wall
        andrew.Stop();
        sleep(200);

        double startTime = getRuntime();
        while(getRuntime()<startTime+1){
            if(!opModeIsActive()) return;
            andrew.TurnTowardsAngle(180,-0.4,0.05);
        }

        sleep(200); // works up to here
        startTime = getRuntime();
        int tickBefore = (int)andrew.getMotorTicks()[0];
        while(andrew.sideDist.getDistance(DistanceUnit.INCH)>10 && getRuntime()<startTime+6){
            if(!opModeIsActive()) return;
        //    telemetry.addData("dist", andrew.sideDist.getDistance(DistanceUnit.INCH));
        //    telemetry.update();
            andrew.RawDrive(180,0.4,0); // scan
        }
        andrew.Stop();
        int distTraveled = (int)andrew.getMotorTicks()[0] - tickBefore;
        int level = 1;        //level 1 is the closest and lowest
        if(distTraveled>650 + 300)
            level = 2;
        if(distTraveled>950 + 300)
            level = 3;


        sleep(1000);

        sleep(200);
        andrew.RawDrive(180,0.6,0);
        waitForMotors(andrew,1900-distTraveled,15);
        andrew.Stop();

sleep(200);
        startTime = getRuntime();  //reorient after next to goal
        while(getRuntime()<startTime+1){
            if(!opModeIsActive()) return;

            telemetry.clear();

            andrew.TurnTowardsAngle(180,-0.4,0.05);
            telemetry.addData("targetHeading",180);
            telemetry.addData("IMU", andrew.GetImu().GetRobotAngle());

            telemetry.update();
        }
        sleep(200);
        andrew.RawDrive(90,0.6,0);
        waitForMotors(andrew,180,2); //left to red line
        andrew.Stop();
        sleep(200);



        int armPosToSet = 5400;
        if(level==1)
            armPosToSet = 7200;
        if(level==2)
            armPosToSet = 6300;


        andrew.armPos.setTargetPosition(armStartPos+armPosToSet);
        andrew.armPos.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        andrew.armPos.setPower(1);

        sleep(2000);

        andrew.turntable.setTargetPosition(turntableStartPos + -3600);
        andrew.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        andrew.turntable.setPower(1);

        sleep(2500);

        //  Drop freight here //
        andrew.gripper.setPower(0);
        sleep(600);


        andrew.turntable.setTargetPosition(turntableStartPos);  //put arm back home
        andrew.turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        andrew.turntable.setPower(1);
        sleep(1000);
        andrew.armPos.setTargetPosition(armStartPos);
        andrew.armPos.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        andrew.armPos.setPower(1);


        sleep(1500);



        andrew.RawDrive(270,0.7,0); //right to park
        waitForMotors(andrew,1000,15);
        andrew.Stop();

        andrew.RawDrive(190,0.8,0.05); //forward to park
        waitForMotors(andrew,5000,15);
        andrew.Stop();



        telemetry.clearAll();
        telemetry.addData("Finished in",getRuntime());
        telemetry.addData("level",level);
        telemetry.update();
        sleep(15000);

*/
    }

    void waitForMotors(AndrewRobot bot, int targetPosition, double timeLimit){
        double startTime = getRuntime();
        int avgDistance = 0;
        int sum = 0;
        int[] initialPositions = new int[4];
        for(int i = 0; i<4; i++)
            initialPositions[i] = (int)bot.getMotorTicks()[i];

        while (avgDistance<targetPosition && getRuntime()<startTime+timeLimit) {
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

    public void wait(double time){
        double startTime = getRuntime();
        while (getRuntime()<startTime+time){
            if(!opModeIsActive())
                return;
        }

    }




}
