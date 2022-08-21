package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;

class DemobotPayload
{
    OpMode opMode;
    MotorArray intake;
    MotorArray path;
    MotorArray loader;
    EncoderActuator turret;
    MotorArray shooter;

    //initializer
    public DemobotPayload(OpMode setOpMode, MotorArray setIntake, MotorArray setPath, MotorArray setLoader, EncoderActuator setTurret, MotorArray setShooter){
        opMode = setOpMode;
        intake = setIntake;
        path = setPath;
        loader = setLoader;
        turret = setTurret;
        shooter = setShooter;

        turret.resetToZero();
        turret.lock();
    }

    ////BASIC FUNCTIONS////
    //turns intake on or off
    public void setIntakeState(boolean on) {
        if(on) intake.setPowers(1);
        else intake.setPowers(0);
    }
    //turns the path on or off
    public void setPathState(boolean on){
        if(on) path.setPowers(1);
        else path.setPowers(0);
    }
    //turns the loader on or off
    public void setLoaderState(boolean on){
        if(on) loader.setPowers(1);
        else loader.setPowers(0);
    }
    //turns the turret to the given angle
    public void setTurretAngle(double degrees){
        turret.goToPosition(degrees);
    }
    //turns the shooter on or off
    public void setShooterState(boolean on) {
        if(on) shooter.setPowers(1);
        else shooter.setPowers(0);
    }

    ////SENSORS////
    public boolean isIntakeFull(){return false;}
    public boolean isPathFull(){return false;}
    public boolean isLoaderFull(){return false;}
}
