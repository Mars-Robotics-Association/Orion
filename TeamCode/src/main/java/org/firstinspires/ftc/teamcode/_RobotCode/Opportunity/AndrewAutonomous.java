package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;


@Config
@Autonomous(name = "Andrew auto", group = "All")
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


    @Override
    public void runOpMode() throws InterruptedException {
        FR = this.hardwareMap.dcMotor.get("FR");
        FL = this.hardwareMap.dcMotor.get("FL");
        RR = this.hardwareMap.dcMotor.get("RR");
        RL = this.hardwareMap.dcMotor.get("RL");

        colorSensor1 = hardwareMap.colorSensor.get("color1");

        duckyMotor = this.hardwareMap.dcMotor.get("duckyMotor");
        imu = new IMU(this);
        andrewIMU = new AndrewIMU(imu);
        imu.Start();
        waitForStart();
    andrewIMU.resetRotation();

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

        duckyMotor.setPower(0-1);

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
        while(getRuntime()<startTime+4){
            if(!opModeIsActive()) return;
            duckyMotor.setPower(0-1);
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

        startTime = getRuntime();

        if(andrewIMU.getRotation()>0&&getRuntime()<startTime+3){
            newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(0,0,0.3);
            while(andrewIMU.getRotation()>0){
                if(!opModeIsActive()) return;
                andrewIMU.loop();
                FR.setPower(newSpeeds[0]);
                FL.setPower(newSpeeds[2]);
                RR.setPower(newSpeeds[1]);
                RL.setPower(newSpeeds[3]);
            }
        }else{
            newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(0,0,-0.3);
            while(andrewIMU.getRotation()<0&&getRuntime()<startTime+3){
                if(!opModeIsActive()) return;
                andrewIMU.loop();
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




        newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(90,0.5,0);
        while((FR.getCurrentPosition()-FRStart)-(FL.getCurrentPosition()-FLStart)<1000*2){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[2]);
            RR.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
        }




    }
}
