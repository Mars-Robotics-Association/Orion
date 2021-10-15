package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoArray
{
    Servo[] servos;
    double[] positions;

    //Provide an array of servos with an array of equal length containing start positions
    public ServoArray(Servo[] setServos, double[] setPositions){
        servos = setServos;
        SetPosArray(setPositions);
    }

    public void SetPosArray(double[] setPositions){
        positions = setPositions;
    }

    public void MoveServosWithMultiplier(double multiplier){
        int i = 0;
        for (Servo s:servos) {
            s.setPosition(positions[i]*multiplier);
            i++;
        }
    }

    public void GoToPositions(double[] setPositions, boolean overrideArray){
        int i = 0;
        for (Servo s:servos) {
            s.setPosition(setPositions[i]);
            i++;
        }

        if(overrideArray) positions = setPositions;
    }

    //Moves all servos to the same position
    public void GoToPosition(double position){
        int i = 0;
        for (Servo s:servos) {
            s.setPosition(position);
            i++;
        }
    }

    //Sets all servo positions to 0.5, stopping continuous rotation servos
    public void StopCRServos(){
        GoToPosition(0.5);
    }
}
