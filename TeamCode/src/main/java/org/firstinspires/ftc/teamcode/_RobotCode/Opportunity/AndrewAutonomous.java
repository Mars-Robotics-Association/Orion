package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

@Config
@Autonomous(name = "Andrew auto", group = "All")
@Disabled
public class AndrewAutonomous extends LinearOpMode
{
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor RR;
    private DcMotor RL;
    private DcMotor[] driveMotors = {FR,FL,RR,RL};
    private IMU imu;
    private AndrewIMU andrewIMU;
    private DcMotor duckyMotor;
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;


    @Override
    public void runOpMode() throws InterruptedException {
        FR = this.hardwareMap.dcMotor.get("FR");
        FL = this.hardwareMap.dcMotor.get("FL");
        RR = this.hardwareMap.dcMotor.get("RR");
        RL = this.hardwareMap.dcMotor.get("RL");

     //   colorSensor1 = hardwareMap.colorSensor.get("color1");
     //   colorSensor2 = hardwareMap.colorSensor.get("color2");

        duckyMotor = this.hardwareMap.dcMotor.get("duckyMotor");
        imu = new IMU(this);
        andrewIMU = new AndrewIMU(imu);
        imu.Start();
        waitForStart();
    andrewIMU.resetRotation();

    andrewIMU.loop();
    double initialRotation = andrewIMU.getRotation();

        int FRStart = FR.getCurrentPosition();
        int FLStart = FL.getCurrentPosition();
        int RRStart = RR.getCurrentPosition();
        int RLStart = RL.getCurrentPosition();

        duckyMotor.setPower(0);
        double startTime = getRuntime();

        double[] newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(270,0.5,0);
        while((FR.getCurrentPosition()-FRStart)-(FL.getCurrentPosition()-FLStart)>-930*2){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[2]);
            RR.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
            telemetry.addData("FR",FR.getCurrentPosition()-FRStart);
            telemetry.addData("FL",FL.getCurrentPosition()-FLStart);
            telemetry.update();
            andrewIMU.loop();

        }
        andrewIMU.loop();
        initialRotation = andrewIMU.getRotation();
        FR.setPower(0);
        FL.setPower(0);
        RR.setPower(0);
        RL.setPower(0);
        newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(180,0.5,0);
        while(colorSensor1.red()<80){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[2]);
            RR.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
            andrewIMU.loop();
        }
        FR.setPower(0);
        FL.setPower(0);
        RR.setPower(0);
        RL.setPower(0);

        FRStart = FR.getCurrentPosition();
        FLStart = FL.getCurrentPosition();
        newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(270,0.5,0);
        while((FR.getCurrentPosition()-FRStart)-(FL.getCurrentPosition()-FLStart)>-200*2){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[2]);
            RR.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
            andrewIMU.loop();
        }
        FR.setPower(0);
        FL.setPower(0);
        RR.setPower(0);
        RL.setPower(0);

        FRStart = FR.getCurrentPosition();
        FLStart = FL.getCurrentPosition();

        duckyMotor.setPower(-0.8);

        startTime = getRuntime();
        newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(0,0.3,0);
        while(getRuntime()<startTime+6&&((FR.getCurrentPosition()-FRStart)-(FL.getCurrentPosition()-FLStart)>-150*2)){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[2]);
            RR.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
            andrewIMU.loop();
        }
        FR.setPower(0);
        FL.setPower(0);
        RR.setPower(0);
        RL.setPower(0);

        startTime = getRuntime();
        while(getRuntime()<startTime+2){
            if(!opModeIsActive()) return;
            duckyMotor.setPower(-1);
        }

        FR.setPower(0);
        FL.setPower(0);
        RR.setPower(0);
        RL.setPower(0);
        newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(180,0.5,0);
        while(colorSensor1.red()<80){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[2]);
            RR.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
            andrewIMU.loop();
        }

        FR.setPower(0);
        FL.setPower(0);
        RR.setPower(0);
        RL.setPower(0);

//        startTime = getRuntime();
//        while(getRuntime()<startTime+6&&Math.abs(andrewIMU.getRotation())>15){
//            andrewIMU.loop();
//            if(!opModeIsActive()) return;
//            if(andrewIMU.getRotation()>0){
//                newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(0,0,0.15);
//            }else{
//                newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(0,0,0-0.15);
//            }
//            FR.setPower(newSpeeds[0]);
//            FL.setPower(newSpeeds[2]);
//            RR.setPower(newSpeeds[1]);
//            RL.setPower(newSpeeds[3]);
//            telemetry.addData("IMU",andrewIMU.getRotation());
//            telemetry.update();
//        }



//        startTime = getRuntime();
//        while(getRuntime()<startTime+15){
//            if(!opModeIsActive()) return;
//            telemetry.addData("angle",andrewIMU.getRotation());
//            telemetry.addData("start",initialRotation);
//            telemetry.update();
//            andrewIMU.loop();
//        }


        boolean turnRight = false;
        startTime = getRuntime();

        andrewIMU.loop();
        if(Math.abs(andrewIMU.getRotation()-initialRotation)>10){
            turnRight = andrewIMU.getRotation()>initialRotation;
            while(andrewIMU.getRotation()>initialRotation!=turnRight){
                if(!opModeIsActive()) return;
                andrewIMU.loop();
                if(turnRight)
                    newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(0,0,0.2);
                else
                    newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(0,0,-0.2);

                FR.setPower(newSpeeds[0]);
                FL.setPower(newSpeeds[2]);
                RR.setPower(newSpeeds[1]);
                RL.setPower(newSpeeds[3]);
            }
        }

        FR.setPower(0);
        FL.setPower(0);
        RR.setPower(0);
        RL.setPower(0);

        FRStart = FR.getCurrentPosition();
        FLStart = FL.getCurrentPosition();


        int distanceTravelled = 0;

        newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(90,0.2,0);
        while((FR.getCurrentPosition()-FRStart)-(FL.getCurrentPosition()-FLStart)<2000*2){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[2]);
            RR.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
            if(colorSensor2.green()>60){
               distanceTravelled = (FR.getCurrentPosition()-FRStart)-(FL.getCurrentPosition()-FLStart)/5;
                break;
            }

        }

        startTime = getRuntime();
        while(getRuntime()<startTime+8){
            if(!opModeIsActive()) return;
            FR.setPower(0);
            FL.setPower(0);
            RR.setPower(0);
            RL.setPower(0);
            telemetry.addData("distance",distanceTravelled);
            telemetry.update();

        }



    }
}
