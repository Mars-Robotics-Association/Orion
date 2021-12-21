package org.firstinspires.ftc.teamcode.Orion.NavModules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Orion.FieldState.Pose;

@Config
public class DriveWheelOdometry
{
    private OpMode opMode;
    private MecanumChassis chassis;
    public static double kX;
    public static double kY;
    public Pose currentPose;
    public int[] lastEncoderTicks;


    public DriveWheelOdometry(MecanumChassis setChassis){
        chassis = setChassis;
        chassis.SetHeadlessMode(true);
        chassis.ResetGyro();
        lastEncoderTicks = chassis.driveMotors.GetMotorPositions();
    }



    /*public void MoveTowards(Pose startPose, Pose endPose, double speed, double turnSpeed){
        //interpolate between a start pose and an end pose
        //f(x) = 2ksin(x)/2PI
        double x = CurrentPose().X / (endPose.X - startPose.X);
        double xSpeed = (2*Math.sin(x))/(2*Math.PI);
        double y = CurrentPose().Y / (endPose.Y - startPose.Y);
        double ySpeed = (2*Math.sin(y))/(2*Math.PI);
        double turnOffset = turnSpeed*(endPose.H - startPose.H);

        opMode.telemetry.addData("Move Angle", Math.toDegrees(Math.atan2(xSpeed,ySpeed)));

        chassis.RawDrive(Math.toDegrees(Math.atan2(xSpeed,ySpeed)), speed, turnOffset);
    }

    public void UpdatePose(){
        //calculate the angle and distance at which the robot moved



        lastEncoderTicks = chassis.driveMotors.GetMotorPositions();
    }

    *//*public double[] CalculateWheelSpeedsTurning(double degrees, double speed, double turnSpeed)
    {
        double FRP = -Math.cos(Math.toRadians(degrees + 45)) * speed + turnSpeed;
        double FLP = Math.cos(Math.toRadians(degrees - 45)) * speed + turnSpeed;
        double RRP = -Math.cos(Math.toRadians(degrees - 45)) * speed + turnSpeed;
        double RLP = Math.cos(Math.toRadians(degrees + 45)) * speed + turnSpeed;

        double[] vals = {FRP, FLP, RRP, RLP};
        return vals;
    }*//*

    public Pose CurrentPose(){
        return ConvertEncoderValsToPose(chassis.driveMotors.GetMotorPositions()[0],0,0);
    }

    public Pose ConvertEncoderValsToPose(double x, double y, double h){
        return new Pose(x*kX, y*kY, h);
    }*/

}
