package org.firstinspires.ftc.teamcode.Navigation;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;

public class EncoderArray
{
    private DCMotorArray motorPorts;
    private double[] multipliers;
    private double ticksPerRotation = 0;
    private double rotationsPerUnit = 0;

    public EncoderArray(DCMotorArray setMotorPorts, double[] setMultipliers, double setTicksPerRotation, double setRotationsPerUnit){
        motorPorts = setMotorPorts;
        multipliers = setMultipliers;
        ticksPerRotation = setTicksPerRotation;
        rotationsPerUnit = setRotationsPerUnit;
    }

    //get positions in the units used
    public double[] getPositions(){
        int[] initialPositions = motorPorts.GetMotorPositions(); //get encoder positions
        double[] positions = new double[initialPositions.length]; //create array
        for (int i = 0; i < positions.length; i++) { //calculate final positions for each value
            positions[i] = ((initialPositions[i]/ticksPerRotation)/rotationsPerUnit)*multipliers[i];
        }
        return positions;
    }
    //stops and resets the encoders
    public void resetEncoders(){motorPorts.StopAndResetEncoders();}
    //set the multipliers for each encoder
    public void setMultipliers(double[] setMultipliers){multipliers = setMultipliers;}

}
