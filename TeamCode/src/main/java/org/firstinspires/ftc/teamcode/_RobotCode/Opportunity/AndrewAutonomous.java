package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    private IMU imu;
    private AndrewIMU andrewIMU;
    private CRServo duckyServo;



    @Override
    public void runOpMode() throws InterruptedException {
        FR = this.hardwareMap.dcMotor.get("FR");
        FL = this.hardwareMap.dcMotor.get("FL");
        RR = this.hardwareMap.dcMotor.get("RR");
        RL = this.hardwareMap.dcMotor.get("RL");
        duckyServo = this.hardwareMap.crservo.get("duckyServo");
        imu = new IMU(this);
        andrewIMU = new AndrewIMU(imu);

        imu.Start();
        waitForStart();
        double[] newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(0,0.5,0);

        double startTime = getRuntime();
        while(getRuntime()<startTime+ 2){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
            RR.setPower(newSpeeds[2]);
        }
        startTime = getRuntime();
        while(getRuntime()<startTime+ 1){
            if(!opModeIsActive()) return;
            FR.setPower(0);
            FL.setPower(0);
            RL.setPower(0);
            RR.setPower(0);
        }

        startTime = getRuntime();
        newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(270,1,0);
        while(getRuntime()<startTime+ 2){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
            RR.setPower(newSpeeds[2]);
        }

        startTime = getRuntime();
        newSpeeds = MecanumChassis.CalculateWheelSpeedsTurning(0,0,0.3);
        while(getRuntime()<startTime+ 15){
            if(!opModeIsActive()) return;
            FR.setPower(newSpeeds[0]);
            FL.setPower(newSpeeds[1]);
            RL.setPower(newSpeeds[3]);
            RR.setPower(newSpeeds[2]);
        }



        FR.setPower(0);
        FL.setPower(0);
        RR.setPower(0);
        RL.setPower(0);


    }
}
