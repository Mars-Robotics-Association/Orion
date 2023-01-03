package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

public class DCMotorArray
{
    DcMotor[] motors;
    public DcMotor[] getMotors(){return motors;}
    double[] speedMultipliers;
    boolean useEncoders = false;

    //Provide an array of motors with an array of equal length which contains speed multipliers
    public DCMotorArray(DcMotor[] setMotors, double[] setSpeedMultipliers, boolean isUseEncoders){
        motors = setMotors;
        speedMultipliers = setSpeedMultipliers;
        useEncoders = isUseEncoders;

        if(useEncoders) runWithEncodersMode();
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
    }

    //Sets the powers of the motors to the same value
    public void setPowers(double speed){
        int i = 0;
        for (DcMotor m: motors) {
            m.setPower(speed*speedMultipliers[i]);
            i++;
        }
    }

    //Stops the motors
    public void stopMotors(){
        setPowers(0);
    }

    public void setTargetPositions(int[] positions, boolean runToPositionsImmediate){
        if(!useEncoders) return;
        int i = 0;
        for (DcMotor m: motors) {
            m.setTargetPosition(positions[i]);
            i++;
        }
        if(runToPositionsImmediate) {
            setPowers(1);
            runToPositionMode();
        }
    }
    public void setTargetPosition(int position, boolean runToPositionImmediate){
        if(!useEncoders) return;
        int i = 0;
        for (DcMotor m: motors) {
            m.setTargetPosition(position);
            i++;
        }
        if(runToPositionImmediate) {
            setPowers(1);
            runToPositionMode();
        }
    }

    public int[] getMotorPositions(){
        List<Integer> positions = new ArrayList<Integer>();
        for(DcMotor m : motors) positions.add(m.getCurrentPosition());
        int[] array = new int[positions.size()];
        for(int i = 0; i < positions.size(); i++) array[i] = positions.get(i);
        return array;
    }

    //Mode setting
    public void runWithEncodersMode(){
        for(DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            useEncoders = true;
        }
    }
    public void runWithoutEncodersMode(){
        for(DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void stopAndResetEncoders(){
        for(DcMotor m : motors) m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runToPositionMode(){
        for(DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
