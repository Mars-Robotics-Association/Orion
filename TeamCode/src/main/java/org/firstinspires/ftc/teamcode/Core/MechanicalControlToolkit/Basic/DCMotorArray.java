package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

public class DCMotorArray
{
    DcMotor[] motors;
    double[] speedMultipliers;
    boolean useEncoders = false;

    //Provide an array of motors with an array of equal length which contains speed multipliers
    public DCMotorArray(DcMotor[] setMotors, double[] setSpeedMultipliers, boolean isUseEncoders){
        motors = setMotors;
        speedMultipliers = setSpeedMultipliers;
        useEncoders = isUseEncoders;

        if(useEncoders) RunWithEncodersMode();
        else RunWithoutEncodersMode();
    }

    public void SetSpeedMultipliers(double[] setSpeedMultipliers){
        speedMultipliers = setSpeedMultipliers;
    }

    //Sets the powers of the motors based off an array using the speedMultipliers array
    public void SetPowers(double[] setSpeeds){
        int i = 0;
        for (DcMotor m: motors) {
            m.setPower(setSpeeds[i]*speedMultipliers[i]);
            i++;
        }
    }

    //Sets the powers of the motors to the same value
    public void SetPowers(double speed){
        int i = 0;
        for (DcMotor m: motors) {
            m.setPower(speed*speedMultipliers[i]);
            i++;
        }
    }

    //Stops the motors
    public void StopMotors(){
        SetPowers(0);
    }

    public void SetTargetPositions(int[] positions, boolean runToPositionsImmediate){
        if(!useEncoders) return;
        int i = 0;
        for (DcMotor m: motors) {
            m.setTargetPosition(positions[i]);
            i++;
        }
        if(runToPositionsImmediate) {
            SetPowers(1);
            RunToPositionMode();
        }
    }
    public void SetTargetPosition(int position, boolean runToPositionImmediate){
        if(!useEncoders) return;
        int i = 0;
        for (DcMotor m: motors) {
            m.setTargetPosition(position);
            i++;
        }
        if(runToPositionImmediate) {
            SetPowers(1);
            RunToPositionMode();
        }
    }

    public int[] GetMotorPositions(){
        List<Integer> positions = new ArrayList<Integer>();
        for(DcMotor m : motors) positions.add(m.getCurrentPosition());
        int[] array = new int[positions.size()];
        for(int i = 0; i < positions.size(); i++) array[i] = positions.get(i);
        return array;
    }

    //Mode setting
    public void RunWithEncodersMode(){
        for(DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            useEncoders = true;
        }
    }
    public void RunWithoutEncodersMode(){
        for(DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void StopAndResetEncoders(){
        for(DcMotor m : motors) m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void RunToPositionMode(){
        for(DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
