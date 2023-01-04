package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;

public class EncoderActuator 
{
    EncoderActuatorProfile profile;
    OpMode opMode;

    //All these values will be set by the profile
    public DCMotorArray motors;
    public double maxRots;
    public double minRots;
    public double gearRatio; //units per revolution
    public double encoderResolution; //encoder ticks per revolution
    public double encoderMultiplier;
    public boolean useEncoder;

    //private
    private double currentTargetPosition = 0;
    
    public EncoderActuator(OpMode setOpMode, EncoderActuatorProfile setProfile){
        profile = setProfile;
        opMode = setOpMode;
        
        //Set off of profile
        motors = profile.motors();
        maxRots = profile.maxRots();
        minRots = profile.minRots();
        gearRatio = profile.gearRatio();
        encoderResolution = profile.encoderResolution();
        if(profile.reverseEncoder()) encoderMultiplier = -1;
        else encoderMultiplier = 1;
        useEncoder = profile.useEncoder();
        resetToZero();
    }

    //Sets the motor to go to a target rotation
    public void goToPosition(double pos){
        motors.setTargetPosition(
                (int)(encoderResolution * clamp(pos,minRots,maxRots) * gearRatio * encoderMultiplier),
                true);
    }

    //Goes to motor's extreme
    public void goToMax(){
        goToPosition(maxRots);
    }

    //Goes to motor's extreme
    public void goToMin(){
        goToPosition(minRots);
    }

    //Resets the motor's encoder
    public void resetToZero(){motors.stopAndResetEncoders();}

    //Sets motor's extreme
    public void resetMax(){maxRots = getPosition();}

    //Locks the motor in place using the encoder
    public void lock(){
        motors.setTargetPositions(motors.getMotorPositions(), true);
    }

    //Sets the motors power and limits its position
    public void setPowerClamped(double power){
        opMode.telemetry.addData("MOTOR POSITION", getPosition());
        opMode.telemetry.addData("REQUESTED POWER", power);
        if(power*encoderMultiplier < 0 && getPosition() < minRots){//+((maxRots-minRots)/10)
            //if(useEncoder) goToPosition(minRots * encoderResolution * gearRatio);
            motors.setPowers(0);
            opMode.telemetry.addData("ADDING POWER", 0);
            return;
        }
        else if(power*encoderMultiplier > 0 && getPosition() > maxRots){//-((maxRots-minRots)/10)
            //if(useEncoder) goToPosition(maxRots * encoderResolution * gearRatio);
            motors.setPowers(0);
            opMode.telemetry.addData("ADDING POWER", 0);
            return;
        }

        opMode.telemetry.addData("ADDING POWER", power);

        motors.runWithEncodersMode();
        motors.setPowers(power);
    }

    public void changeCurrentTargetRotation(double deltaAmount, double speed){
        double newPos = currentTargetPosition + deltaAmount;
        if(newPos > maxRots) {
            goToPosition(maxRots);
            return;
        }
        else if(newPos < minRots) {
            goToPosition(minRots);
            return;
        }
        motors.setPowers(speed);
        goToPosition(newPos);
    }
    public void changeCurrentTargetRotation(double deltaAmount, double speed, double min, double max){
        double newPos = currentTargetPosition + deltaAmount;
        if(newPos > maxRots || newPos > max) {
            goToPosition(maxRots);
            return;
        }
        else if(newPos < minRots || newPos < min) {
            goToPosition(minRots);
            return;
        }
        motors.setPowers(speed);
        goToPosition(newPos);
    }

    //Set the motors power freely
    public void setPowerRaw(double power){
        opMode.telemetry.addData("MOTOR POSITION", getPosition());
        opMode.telemetry.addData("REQUESTED POWER", power);
        motors.runWithEncodersMode();
        motors.setPowers(power);
    }

    //Returns the rotation of the final implement
    public double getPosition(){
        return motors.getMotorPositions()[0] * (encoderMultiplier / (encoderResolution * gearRatio));
    }
    public double getPosition(int motorIndex){
        return motors.getMotorPositions()[motorIndex] * (encoderMultiplier / (encoderResolution * gearRatio));
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
