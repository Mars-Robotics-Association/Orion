package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis;

//REQUIRED TO COMPILE: Phones | REV Hub
//REQUIRED TO RUN: Chassis

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;
import org.firstinspires.ftc.teamcode.Core.PIDController;

public class TankChassis
{
    ////Dependencies////
    private OpMode CurrentOpMode;
    private PIDController headingPIDController;
    private IMU imu;

    ////Variables////
    //Motors
    MotorArray driveMotors;
    private boolean reverseHeadingPID = false;

    //Brake pos
    private int RBrakePos = 0;
    private int LBrakePos = 0;

    //telemetry
    private Telemetry telemetry;

    public PIDController GetHeadingPID() {
        return headingPIDController;
    }

    //Initializer
    public TankChassis(IMU setImu, DcMotor r, DcMotor l, Telemetry telemetry, boolean useEncoders){
        imu = setImu;

        driveMotors = new MotorArray(new DcMotor[]{r,l}, new double[]{1,1}, useEncoders);

        this.telemetry = telemetry;
        headingPIDController = new PIDController(0,0,0);//Create the pid controller.

    }

    public void SetHeadingPID(double p, double i, double d, boolean reverse){
        headingPIDController.setPID(p,i,d);
        reverseHeadingPID = reverse;
    }

    //Function for normal driving. Speed is generally y-axis input on joystick and turnFactor is x-axis, both between -1 and 1
    public void DriveNormal(double speed, double turnFactor){
        driveMotors.SetPowers(new double[]{speed+turnFactor, speed-turnFactor});
    }

    //Function for headless movement. Takes in a target direction and speed as well as a spot turn value for static turning
    public void DriveHeadless(double speed, double targetHeading, double spotTurnFactor, double sweepTurnFactor){
        if(speed < 0.05){ //only do a spot turn and return
            driveMotors.SetPowers(new double[]{spotTurnFactor, -spotTurnFactor});
            return;
        }
        //if the speed is greater than the threshold, do lateral movement
        //Uses pid controller to correct for heading error using (currentAngle, targetAngle)
        //TODO: fix issue where when heading is around 180 and -180 it goes into crazy spins
        double headingPIDOffset = headingPIDController.getOutput(targetHeading, imu.GetRobotAngle());
        //if the number is not real, reset pid controller
        if(!(headingPIDOffset > 0 || headingPIDOffset <= 0)){
            headingPIDController.reset();
        }

        if(reverseHeadingPID) headingPIDOffset *= -1;

        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Actual Heading", imu.GetRobotAngle());
        telemetry.addData("Heading PID", headingPIDOffset);
        telemetry.addData("R speed", speed+headingPIDOffset+sweepTurnFactor);
        telemetry.addData("L speed", speed-headingPIDOffset-sweepTurnFactor);

        driveMotors.SetPowers(new double[]{speed+headingPIDOffset+sweepTurnFactor, speed-headingPIDOffset-sweepTurnFactor});
    }

}
