package org.firstinspires.ftc.teamcode.Orion.NavModules;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;


public class WallFollower {
    private OpMode opMode;
    private MecanumChassis chasis;
    private Motor.Encoder encoder;
    public WallFollower(OpMode opmode, MecanumChassis passedChasis, Motor.Encoder passedEncoder){
        opMode = opmode;
        chasis = passedChasis;
        encoder = passedEncoder;

    }
    public void followEncoder(boolean onLeft){
        encoder.reset();
        chasis.RawDrive(0,0.5,0);

    }

}