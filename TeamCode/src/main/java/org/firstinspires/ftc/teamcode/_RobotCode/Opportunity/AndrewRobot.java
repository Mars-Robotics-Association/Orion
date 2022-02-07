package org.firstinspires.ftc.teamcode._RobotCode.Opportunity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AndrewRobot extends MecanumChassis {
    DcMotor duckyMotor;
    DistanceSensor sideDist;

    DistanceSensor rotationDist1;
    DistanceSensor rotationDist2;

    DcMotor armPos;
    DcMotor turntable;
    DcMotor gripper;
    LinearOpMode linearOpMode;

    public void init(){
        duckyMotor = super.opMode.hardwareMap.dcMotor.get("duckyMotor");
        sideDist = super.opMode.hardwareMap.get(DistanceSensor.class, "distSide");
        armPos = super.opMode.hardwareMap.dcMotor.get("armPosition");
        turntable = super.opMode.hardwareMap.dcMotor.get("turntable");
        gripper = super.opMode.hardwareMap.dcMotor.get("clawMotor");

        rotationDist1 = super.opMode.hardwareMap.get(DistanceSensor.class, "rotDist1");
        rotationDist2 = super.opMode.hardwareMap.get(DistanceSensor.class, "rotDist2");


    }

    public AndrewRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator, LinearOpMode linearOpMode) {
        super(setOpMode, new _ChassisProfile(), new HermesLog("Opportunity", 500, setOpMode), useChassis, usePayload, useNavigator);
        this.linearOpMode = linearOpMode;
    }

    public void wait(double time){
        double startTime = opMode.getRuntime();
        while (opMode.getRuntime()<startTime+time){
            if(!linearOpMode.opModeIsActive()) return;

        }

    }

    public void start(){
        super.StartCoreRobotModules();
    }




    public double TurnTowardsAngle2(double ld_TargetHeading, double ld_Speed, double ld_Coefficient){
        // Turn toward the angle.
            // ld_TargetHeading = the desired heading.  Range: -180 to 180
            // ld_Speed = the desired speed.  Range: 0 to 1
            // ld_Coefficient = adjustment coefficient.  Range: >0 to 1

            /* if(ld_TargetHeading > 180) ld_TargetHeading = -360+ld_TargetHeading;
                else if(ld_TargetHeading < -180) ld_TargetHeading = 360+ld_TargetHeading;
                //calculate error and turn speed
                double error = ld_TargetHeading - imu.GetRobotAngle();
            */

        // Get the difference between the desired heading and the robots current orientation.
        double ld_error = GetTargetHeadingDifference(ld_TargetHeading);

        // If ld_error = 0, then abort before a divide by zero error...
        if (ld_error == 0){
            return ld_error;
        }
        // Calculate the TurnSpeed
        double ld_TurnSpeed = (ld_error*ld_Coefficient*ld_Speed) + 0.2*(ld_error/Math.abs(ld_error));

        // Turn the Robot
        this.RawTurn(ld_TurnSpeed);

        // Get the new difference (post-turn) between the desired heading and the robots current orientation.
         ld_error = GetTargetHeadingDifference(ld_TargetHeading);

        return ld_error;
    }

    public double GetTargetHeadingDifference (double ld_TargetHeading){
        // Calculate the difference between TargetHeading and the robot's current orientation.
        // ld_TargetHeading = the desired heading.  Range: -180 to 180
        //

        // Error check ld_TargetHeading
        // If ld_TargetHeading is greater than 180 degrees, convert to negative value.
        // EX:  190 = -170
        // If ld_TargetHeading is less than -180 degrees, convert to positive value.
        // EX:  -190 = 170
        if(ld_TargetHeading > 180) ld_TargetHeading = -360+ld_TargetHeading;
        else if(ld_TargetHeading < -180) ld_TargetHeading = 360+ld_TargetHeading;

        //Calculate error and turn speed
        double ld_error = ld_TargetHeading - imu.GetRobotAngle();

        // Error check ld_error.
        // If ld_error is greater than 180 degrees, convert to negative value.
        // EX:  190 = -170
        // If ld_error is less than -180 degrees, convert to positive value.
        // EX:  -190 = 170
        if(ld_error > 180) ld_error = -360+ld_error;

        return ld_error;
    }

    public double TurnToAngle(double ld_TargetHeading, double ld_Speed, double ld_Precision){
        // Turn the Robot to the specified angle.
        // ld_TargetHeading = the desired heading.  Range: -180 to 180
        // ld_Speed = the desired speed.  Range: 0 to 1
        // ld_Precision = the number of degrees =/- of ld_TargetHeading that is "close enough"

        double ld_startTime = opMode.getRuntime();
        int li_iteration = 0;
        int li_MaxIterations=6;
        boolean lb_positive;

        // Turn the Robot.
        // Get the Current Heading Difference
        double ld_error = GetTargetHeadingDifference (ld_TargetHeading);

        // Set whether the current error is positive (true) or negative (false)
        lb_positive = (ld_error > 0);

        // While the robot is not within the desired precision and has not exceeded the attempts limit
        while (Math.abs(ld_error)>=ld_Precision && li_iteration<li_MaxIterations){
            if(!linearOpMode.opModeIsActive()) return ld_error;
            ld_error= TurnTowardsAngle2(ld_TargetHeading, ld_Speed, 0.05);
        // If we overshoot the correction, cut the spped in half and try again.
            if (lb_positive != (ld_error > 0)){
                ld_Speed /= 2;
                li_iteration++;
            }
        }

        return ld_error;
    }

    public double CalculateAngleFromTwoDistances(double ld_Dist1, double ld_Dist2,double ld_DistBetween){
// Calculate the angle of an object based on two measured distances and the distance between measurements.
// EX:  Calculate the angle of a robot relative to the game field wall based on the distances returned from two distance sensors and the distance beween those sensors.

// tan(Theta) = Opposite / Adjacent

        double ld_adjacent = ld_DistBetween;
        double ld_opposite = Math.abs(ld_Dist1-ld_Dist2);
        double ld_theta = Math.atan(ld_opposite/ld_adjacent);

        return ld_theta;
    }

    public double getRotationFromDistanceSensors(){
        return CalculateAngleFromTwoDistances(rotationDist1.getDistance(DistanceUnit.INCH),rotationDist2.getDistance(DistanceUnit.INCH),7.75);
    }




}
