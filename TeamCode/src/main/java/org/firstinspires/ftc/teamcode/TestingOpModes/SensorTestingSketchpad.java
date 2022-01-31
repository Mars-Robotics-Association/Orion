package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor Testing Sketchpad", group = "Test")
@Disabled
public class SensorTestingSketchpad extends OpMode
{
    DistanceSensor sensorPort;
    DistanceSensor sensorStarboard;
    ColorSensor colorSensor;

    @Override
    public void init() {
        sensorPort = hardwareMap.get(DistanceSensor.class, "distancePort");
        sensorStarboard = hardwareMap.get(DistanceSensor.class, "distanceStarboard");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
    }

    @Override
    public void loop() {
        telemetry.addData("PORT Distance CM", sensorPort.getDistance(DistanceUnit.CM));
        telemetry.addData("STARBOARD Distance CM", sensorStarboard.getDistance(DistanceUnit.CM));
        telemetry.addData("Color Sensor Alpha", colorSensor.alpha());
        telemetry.update();

    }
}
