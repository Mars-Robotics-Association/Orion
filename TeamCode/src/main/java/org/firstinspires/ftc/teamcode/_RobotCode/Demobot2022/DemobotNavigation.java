package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.EncoderArray;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.*;

@Config
class DemobotNavigation
{
    ////DEPENDENCIES////
    private MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}
    private HolonomicOdometry odometry;
    private EncoderArray encoders;
    ////CONFIGURABLE////
    public static double[] encoderMultipliers =  new double[]{1,1,1};
    public static double trackwidth = 16;
    public static double centerWheelOffset = 6;

    public DemobotNavigation(OpMode setOpMode, BaseRobot baseRobot){
        chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("Demobot", 200, setOpMode), baseRobot);
        odometry = new HolonomicOdometry(trackwidth,centerWheelOffset);

        //get the drive motors in order (LEFT, RIGHT, HORIZONTAL) encoder
        DcMotor[] driveMotors = new DcMotor[]{
                chassis.driveMotors.getMotors()[0],
                chassis.driveMotors.getMotors()[1],
                chassis.driveMotors.getMotors()[2]};

        //creates the encoder array
        encoders = new EncoderArray(
                new DCMotorArray(driveMotors,new double[]{1,1,1},true),
                encoderMultipliers, 8192, 0.5);

    }

    public void update(){
        odometry.update(getDeadWheelPositions()[0], getDeadWheelPositions()[1], getDeadWheelPositions()[2]);
    }

    //gets the positions of the dead wheels
    public double[] getDeadWheelPositions(){return encoders.getPositions();}
    public void setRobotPose(double x, double y, double angle){odometry.updatePose(new Pose2d(x,y,angle));}
    public Pose2d getPose(){return odometry.getPose();}

}
