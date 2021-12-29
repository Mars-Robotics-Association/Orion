package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;
import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavTuningProfile;

class _NavTuningProfile implements FreightFrenzyNavTuningProfile
{
    //Put references to sensors and controllers here
    UniversalTurretIntakeArm arm;
    DuckSpinner spinner;
    DistanceSensor duckDistance;
    DistanceSensor intakeDistance;
    TouchSensor portTouch;
    TouchSensor starboardTouch;
    ColorSensor colorSensor;

    //Put tuning variables for field navigation here

    public _NavTuningProfile(UniversalTurretIntakeArm setArm, DuckSpinner setSpinner, DistanceSensor setDuckDist, DistanceSensor setIntakeDist, TouchSensor setPortTouch, TouchSensor setStarboardTouch, ColorSensor setColorSensor){
        arm = setArm;
        spinner = setSpinner;
        duckDistance = setDuckDist;
        intakeDistance = setIntakeDist;
        portTouch = setPortTouch;
        starboardTouch = setStarboardTouch;
        colorSensor = setColorSensor;
    }

    @Override
    public UniversalTurretIntakeArm arm() {
        return arm;
    }

    @Override
    public DuckSpinner spinner() {
        return spinner;
    }

    @Override
    public DistanceSensor duckDistance() {
        return duckDistance;
    }

    @Override
    public DistanceSensor intakeDistance() {
        return intakeDistance;
    }

    @Override
    public TouchSensor portTouch() {
        return portTouch;
    }

    @Override
    public TouchSensor starboardTouch() {
        return starboardTouch;
    }

    @Override
    public ColorSensor colorSensor() {
        return colorSensor;
    }
}
