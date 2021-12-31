package org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;
import org.firstinspires.ftc.teamcode._RobotCode.Curiosity.DuckSpinner;

public interface FreightFrenzyNavTuningProfile
{
    //Put references to sensors and controllers here
    UniversalTurretIntakeArm arm();
    DuckSpinner spinner();
    DistanceSensor duckDistance();
    DistanceSensor intakeDistance();
    TouchSensor portTouch();
    TouchSensor starboardTouch();
    ColorSensor colorSensor();

    //Put tuning variables for field navigation here

}
