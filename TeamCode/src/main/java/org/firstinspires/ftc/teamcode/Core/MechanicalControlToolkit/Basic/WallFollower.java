package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

public class WallFollower {

    double ticksPerRotation = 100;
    double rotationsPerInch = 1;

    MecanumChassis chassis;
    public WallFollower(MecanumChassis chassisIn){
    chassis = chassisIn;
    }
    public void followWall(boolean onLeft, double inches){
        double ticksToMove = inches*ticksPerRotation/rotationsPerInch;
        double[] tickAr = chassis.getMotorTicks();
        double startPos = tickAr[0];

        double driveAngle = Math.toRadians(4);
        if(onLeft) driveAngle = 0-driveAngle;

        while(Math.abs(startPos-tickAr[0])<ticksToMove){
          chassis.RawDrive(driveAngle,1,0);
            tickAr = chassis.getMotorTicks();
        }

    }

}
