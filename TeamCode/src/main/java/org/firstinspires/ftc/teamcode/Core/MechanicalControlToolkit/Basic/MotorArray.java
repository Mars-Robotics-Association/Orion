package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorArray
{
    DcMotor[] motors;
    double[] speedMultipliers;
    boolean useEncoders = false;

    //Provide an array of motors with an array of equal length which contains speed multipliers
    public MotorArray(DcMotor[] setMotors, double[] setSpeedMultipliers, boolean isUseEncoders){
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
    public void SetMotorPowers(double[] setSpeeds){
        int i = 0;
        for (DcMotor m: motors) {
            m.setPower(setSpeeds[i]*speedMultipliers[i]);
            i++;
        }
    }

    //Sets the powers of the motors to the same value
    public void SetMotorPowers(double speed){
        int i = 0;
        for (DcMotor m: motors) {
            m.setPower(speed*speedMultipliers[i]);
            i++;
        }
    }

    //Sets all servo positions to 0.5, stopping continuous rotation servos
    public void StopMotors(){
        SetMotorPowers(0);
    }

    public void SetTargetPositions(int[] positions, boolean runToPositionsImmediate){
        if(!useEncoders) return;
        int i = 0;
        for (DcMotor m: motors) {
            m.setTargetPosition(positions[i]);
            i++;
        }
        if(runToPositionsImmediate) RunToPositionMode();
    }

    public void SetTargetPositions(int[] positions, boolean runToPositionsImmediate, boolean resetEncoders){
        if(!useEncoders) return;
        if(resetEncoders) StopAndResetEncoders();
        int i = 0;
        for (DcMotor m: motors) {
            m.setTargetPosition(positions[i]);
            i++;
        }
        if(runToPositionsImmediate) RunToPositionMode();
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
