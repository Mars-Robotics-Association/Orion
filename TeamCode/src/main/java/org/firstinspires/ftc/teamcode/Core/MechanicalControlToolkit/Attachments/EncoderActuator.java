package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;

public class EncoderActuator 
{
    EncoderActuatorProfile profile;
    OpMode opMode;

    //All these values will be set by the profile
    public MotorArray motors;
    public double maxRots;
    public double minRots;
    public double gearRatio;
    public double encoderResolution;
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
        ResetToZero();
    }

    //Sets the motor to go to a target rotation
    public void GoToPosition(double pos){
        motors.RunWithEncodersMode();
        motors.SetTargetPosition((int)(encoderResolution * clamp(pos,minRots,maxRots) * gearRatio * encoderMultiplier), true);
    }

    //Goes to motor's extreme
    public void GoToMax(){
        GoToPosition(maxRots);
    }

    //Resets the motor's encoder
    public void ResetToZero(){motors.StopAndResetEncoders();}

    //Sets motor's extreme
    public void ResetMax(){maxRots = GetFinalPosition();}

    //Locks the motor in place using the encoder
    public void Lock(){
        motors.SetTargetPositions(motors.GetMotorPositions(), true);
    }

    //Sets the motors power and limits its position
    public void SetPowerClamped(double power){
        opMode.telemetry.addData("MOTOR POSITION", GetFinalPosition());
        opMode.telemetry.addData("REQUESTED POWER", power);
        if(power*encoderMultiplier < 0 && GetFinalPosition() < minRots){
            if(useEncoder) GoToPosition(minRots * encoderResolution * gearRatio);
            else motors.SetPowers(0);
            opMode.telemetry.addData("ADDING POWER", 0);
            return;
        }
        else if(power*encoderMultiplier > 0 && GetFinalPosition() > maxRots){
            if(useEncoder) GoToPosition(maxRots * encoderResolution * gearRatio);
            else motors.SetPowers(0);
            opMode.telemetry.addData("ADDING POWER", 0);
            return;
        }

        opMode.telemetry.addData("ADDING POWER", power);

        motors.RunWithEncodersMode();
        motors.SetPowers(power);
    }

    public void ChangeCurrentTargetRotation(double deltaAmount, double speed){
        double newPos = currentTargetPosition + deltaAmount;
        if(newPos > maxRots) {
            GoToPosition(maxRots);
            return;
        }
        else if(newPos < minRots) {
            GoToPosition(minRots);
            return;
        }
        motors.SetPowers(speed);
        GoToPosition(newPos);
    }
    public void ChangeCurrentTargetRotation(double deltaAmount, double speed, double min, double max){
        double newPos = currentTargetPosition + deltaAmount;
        if(newPos > maxRots || newPos > max) {
            GoToPosition(maxRots);
            return;
        }
        else if(newPos < minRots || newPos < min) {
            GoToPosition(minRots);
            return;
        }
        motors.SetPowers(speed);
        GoToPosition(newPos);
    }

    //Set the motors power freely
    public void SetPowerRaw(double power){
        motors.RunWithEncodersMode();
        motors.SetPowers(power);
    }

    //Returns the rotation of the final implement
    public double GetFinalPosition(){
        return motors.GetMotorPositions()[0] * (encoderMultiplier / (encoderResolution * gearRatio));
    }
    public double GetFinalPosition(int motorIndex){
        return motors.GetMotorPositions()[motorIndex] * (encoderMultiplier / (encoderResolution * gearRatio));
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
