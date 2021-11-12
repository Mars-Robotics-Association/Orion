package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic;

public class ColorSensor
{
    com.qualcomm.robotcore.hardware.ColorSensor sensor;

    public ColorSensor(com.qualcomm.robotcore.hardware.ColorSensor setSensor){
        sensor = setSensor;
    }

    public int GetReflectedLight(){
        return sensor.alpha();
    }
}
