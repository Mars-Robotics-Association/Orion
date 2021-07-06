package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.List;

public class ColorSensorArray
{
    private OpMode opMode;
    private List<ColorSensor> sensors;

    public ColorSensorArray(OpMode setOpMode, String[] hardwareMapNames){
        opMode = setOpMode;
        for (String name : hardwareMapNames) {
            sensors.add(opMode.hardwareMap.colorSensor.get(name));
        }
    }

    public ColorSensor GetSensor(int index){return sensors.get(index);}
}
