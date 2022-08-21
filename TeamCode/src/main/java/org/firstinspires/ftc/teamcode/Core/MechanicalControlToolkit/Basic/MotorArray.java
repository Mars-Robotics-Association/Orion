package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class MotorArray
{
    DcMotor[] motors;
    Servo[] servos;
    double[] speedMultipliers;
    double[] servoPositions;
    boolean useDCMotorEncoders = false;

    //Provide an array of motors with an array of equal length which contains speed multipliers
    public MotorArray(DcMotor[] setMotors, Servo[] setServos, double[] setSpeedMultipliers, boolean isUseDCEncoders){
        motors = setMotors;
        servos = setServos;
        speedMultipliers = setSpeedMultipliers;
        useDCMotorEncoders = isUseDCEncoders;

        if(useDCMotorEncoders) runWithEncodersMode();
        else runWithoutEncodersMode();
    }

    public void setSpeedMultipliers(double[] setSpeedMultipliers){
        speedMultipliers = setSpeedMultipliers;
    }

    //Sets the powers of the motors based off an array using the speedMultipliers array
    public void setPowers(double[] setSpeeds){
        int i = 0;
        for (DcMotor m: motors) {
            m.setPower(setSpeeds[i]*speedMultipliers[i]);
            i++;
        }
        for (Servo s: servos) {
            s.setPosition(convertMotorSpeedToServoSpeed(setSpeeds[i]*speedMultipliers[i]));
            i++;
        }
    }

    //Sets the powers of the motors to the same value
    public void setPowers(double speed){
        int i = 0;
        for (DcMotor m: motors) {
            m.setPower(speed*speedMultipliers[i]);
            i++;
        }
        for (Servo s: servos) {
            s.setPosition(convertMotorSpeedToServoSpeed(speed*speedMultipliers[i]));
            i++;
        }
    }

    //Stops the motors
    public void stopMotors(){
        setPowers(0);
    }

    //set and possibly go to target positions
    public void setTargetPositions(int[] positions, boolean runToPositionsImmediate){
        int i = 0;
        //servos
        for (Servo s:servos) {
            s.setPosition(clamp(positions[i], 0, 1));
            i++;
        }
        //dc motors
        if(!useDCMotorEncoders) return;
        for (DcMotor m: motors) {
            m.setTargetPosition(positions[i]);
            i++;
        }
        //if turning the motors immediately to position
        if(runToPositionsImmediate) {
            setPowers(1);
            runToPositionMode();
        }
    }
    //set and go to a target position
    public void setTargetPosition(int position, boolean runToPositionImmediate){
        int i = 0;
        //servos
        for (Servo s:servos) {
            s.setPosition(clamp(position, 0, 1));
            i++;
        }
        //dc motors
        if(!useDCMotorEncoders) return;
        for (DcMotor m: motors) {
            m.setTargetPosition(position);
            i++;
        }
        //if turning the motors immediately to position
        if(runToPositionImmediate) {
            setPowers(1);
            runToPositionMode();
        }
    }

    //Return all the current positions of servos and motors
    public double[] getMotorPositions(){
        //create blank list
        List<Double> positions = new ArrayList<Double>();
        //add motor positions in encoder ticks
        for (DcMotor m : motors) positions.add((double) m.getCurrentPosition());
        //add servos positions in values between 0 and 1
        for (Servo s : servos) positions.add(s.getPosition());
        //convert list to array
        double[] array = new double[positions.size()];
        for(int i = 0; i < positions.size(); i++) array[i] = positions.get(i);
        return array;
    }

    //Convert speed between -1 and 1 to between 0 and 1 for servos
    private double convertMotorSpeedToServoSpeed(double motorSpeed){
        motorSpeed = clamp(motorSpeed, -1, 1); //clamp speed between -1 and 1
        double servoSpeed = (motorSpeed+1)/2; //convert
        return servoSpeed;
    }

    //DC MOTOR MODE SETTING
    public void runWithEncodersMode(){
        for(DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            useDCMotorEncoders = true;
        }
    }
    public void runWithoutEncodersMode(){ for(DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); }
    public void stopAndResetEncoders(){ for(DcMotor m : motors) m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }
    public void runToPositionMode(){ for(DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_TO_POSITION); }

    //Clamps a value between a max and a min value
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
