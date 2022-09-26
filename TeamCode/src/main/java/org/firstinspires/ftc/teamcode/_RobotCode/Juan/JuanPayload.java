package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;

class JuanPayload
{
    OpMode opMode;
    MotorArray intake;
    MotorArray path;
    MotorArray loader;
    EncoderActuator turret;
    MotorArray shooter;

    //internal
    boolean intakeOn = false;
    boolean pathOn = false;
    boolean shooterOn = false;

    //initializer
    public JuanPayload(OpMode setOpMode){
        opMode = setOpMode;
    }


    ////BASIC FUNCTIONS////

    //INTAKE//
    //turns intake on or off
    public void setIntakeState(boolean on) {
        if(on) {
            //turn intake on
            intake.setPowers(1);
        }
        else intake.setPowers(0);
        intakeOn = on;
    }
    //toggles intake on and off
    public void toggleIntake(){
        if(intakeOn) setIntakeState(false);
        else setIntakeState(true);
    }

    //PATH//
    //turns the path on or off
    public void setPathState(boolean on){
        if(on) path.setPowers(1);
        else path.setPowers(0);
        pathOn = on;
    }
    //toggles path on and off
    public void togglePath(){
        if(pathOn) setPathState(false);
        else setPathState(true);
    }

    //LOADER//
    //turns the loader on or off
    public void setLoaderState(boolean on){
        if(on) {
            //turn off intake
            setPathState(false);
            setIntakeState(false);
            //turn on loader
            loader.setPowers(1);
        }
        else loader.setPowers(0);
    }

    //TURRET//
    //rotates turret within bounds
    public void moveTurret(double speed){
        turret.setPowerClamped(speed);
    }
    //turns the turret to the given angle
    public void setTurretAngle(double degrees){
        turret.goToPosition(degrees);
    }

    //SHOOTER//
    //turns the shooter on or off
    public void setShooterState(boolean on) {
        if(on) shooter.setPowers(1);
        else shooter.setPowers(0);
        shooterOn = on;
    }
    //toggles shooter on and off
    public void toggleShooter(){
        if(shooterOn) setShooterState(false);
        else setShooterState(true);
    }

    //print telemetry
    public void printTelemetry(){
        opMode.telemetry.addLine("----PAYLOAD----");
        opMode.telemetry.addData("intake on: ", intakeOn);
        opMode.telemetry.addData("path on: ", pathOn);
        opMode.telemetry.addData("shooter on: ", shooterOn);

    }

    ////SENSORS////
    public boolean isIntakeFull(){return false;}
    public boolean isPathFull(){return false;}
    public boolean isLoaderFull(){return false;}
}
