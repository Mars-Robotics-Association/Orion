package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Navigation.PurePursuit.path.*;


class DemobotNavigation
{
    private MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}
    private HolonomicOdometry odometry;
    private Path currentPath;

    public DemobotNavigation(OpMode setOpMode, BaseRobot baseRobot){
        chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("Demobot", 200, setOpMode), baseRobot);
        odometry = new HolonomicOdometry(16,6);
    }

    public void update(){
        currentPath.update(odometry.getPose());
    }

    public void FollowPath(Path path){
        if(!currentPath.equals(path)) currentPath = path;
        currentPath.update(odometry.getPose()); //update robot's pose along path
        Pose2d move = path.getFollowPose(); //get move vector
        chassis.RawDriveTurningTowards(Math.atan2(move.getX(), move.getY()), 0.5, move.getHeading(), 0.01);
    }
}
