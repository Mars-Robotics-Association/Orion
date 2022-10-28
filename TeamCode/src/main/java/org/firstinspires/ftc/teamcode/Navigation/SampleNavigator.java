package org.firstinspires.ftc.teamcode.Navigation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;

public class SampleNavigator extends UniversalThreeWheelNavigator
{

    public SampleNavigator(OpMode setOpMode, BaseRobot baseRobot, DistanceSensor setDistancePort, DistanceSensor setDistanceStarboard, ColorSensor setColorSensor) {

        ////CONFIGURABLE////
        encoderMultipliers = new double[]{1, 1, -1}; //left right horizontal
        trackwidth = 10.8;
        centerWheelOffset = -3;

        turnPID_P = 0.035;
        turnPID_I = 0;
        turnPID_D = -0.1;

        movePID_D = 0.1;
        movePID_I = 0;
        movePID_P = 0.3;

        stopSpeedThreshold = 0.1; //how slow the robot needs to be moving before it stops
        stopTimeThreshold = 0.2; //how long it needs to be below speed threshold

        InitializeNavigator(setOpMode, baseRobot, setDistancePort, setDistanceStarboard, setColorSensor);
    }
}
