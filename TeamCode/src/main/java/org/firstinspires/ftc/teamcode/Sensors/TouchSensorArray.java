package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.List;

public class TouchSensorArray
{
    private OpMode opMode;
    private List<TouchSensor> sensors;

    public TouchSensorArray(OpMode setOpMode, String[] hardwareMapNames){
        opMode = setOpMode;
        for (String name : hardwareMapNames) {
            sensors.add(opMode.hardwareMap.touchSensor.get(name));
        }
    }

    public TouchSensor GetSensor(int index){return sensors.get(index);}
}
