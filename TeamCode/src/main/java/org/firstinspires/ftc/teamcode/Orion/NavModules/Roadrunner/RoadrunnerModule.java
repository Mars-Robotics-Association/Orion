package org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Orion.Archive.NavProfiles.NavigationProfile;
import org.firstinspires.ftc.teamcode.Orion.FieldState.FieldObject;
import org.firstinspires.ftc.teamcode.Orion.FieldState.FieldStateWrapper;
import org.firstinspires.ftc.teamcode.Orion.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.StandardMecanumDrive;

public class RoadrunnerModule
{
    private OpMode opMode;
    private StandardMecanumDrive drive;
    private FieldStateWrapper fieldState;

    public RoadrunnerModule(OpMode setOpMode, RoadrunnerRobotProfile setRobotProfile, RoadrunnerTuningProfile setTuningProfile, FieldStateWrapper setFieldState){
        opMode = setOpMode;
        drive = new StandardMecanumDrive(opMode.hardwareMap, setRobotProfile, setTuningProfile);
        fieldState = setFieldState;
    }

    public void Update(){
        drive.update();
        FieldObject robotFieldObject = new FieldObject("rr_robot_pose", "odometry", FieldObject.ObjectType.SEMISTATIC, new Pose(0,0,0), new Pose(GetCurrentPose().getX(), GetCurrentPose().getY(), GetCurrentPose().getHeading()), 0.8, 0);
        fieldState.AddObjects(new FieldObject[]{robotFieldObject}, opMode);
    }

    public void MoveSpline(double x, double y, double tangent, boolean reverse){ //moves using fancy splines
        Trajectory traj = drive.trajectoryBuilder(GetCurrentPose(), reverse)
                .splineTo(new Vector2d(x, y), tangent)
                .build();

        drive.followTrajectory(traj);
    }

    public void MoveLine(double x, double y, double heading){ //moves linearly along a line .lineToLinearHeading(new Pose2d(x,y,heading))
        Trajectory traj = drive.trajectoryBuilder(GetCurrentPose())
                .lineToLinearHeading(new Pose2d(x,y,heading))
                .build();

        drive.followTrajectory(traj);
    }

    public void MoveLine(double x, double y){ //moves linearly along a line .lineToLinearHeading(new Pose2d(x,y,heading))
        Trajectory traj = drive.trajectoryBuilder(GetCurrentPose())
                .lineTo(new Vector2d(x,y))
                .build();

        drive.followTrajectory(traj);
    }

    public void MoveSplineConstantHeading(double x, double y, double endTangent, boolean reverse){
        Trajectory traj = drive.trajectoryBuilder(GetCurrentPose(), reverse)
                .splineToConstantHeading(new Vector2d(x,y), endTangent)
                .build();

        drive.followTrajectory(traj);
    }

    public void MoveRaw(Pose2d move){
        drive.setDrivePower(move);
    }
    public void TurnRaw(double speed){
        drive.setMotorPowers(speed, speed, -speed, -speed);
    }

    public void Turn(double angle){drive.turn(Math.toRadians(angle));}
    public void TurnTo(double angle){
        if(angle > 360) angle -= 360;
        else if(angle < 0) angle += 360;

        double turnDegrees = angle - Math.toDegrees(GetCurrentPose().getHeading());
        if(turnDegrees > 180) turnDegrees -= 360;
        if(turnDegrees < -180) turnDegrees += 360;

        Turn(turnDegrees);
    }

    public void SetPose(double x, double y, double heading){drive.setPoseEstimate(new Pose2d(x,y, heading));} //Sets robot pose
    public Pose2d GetCurrentPose(){return drive.getPoseEstimate();}

}
