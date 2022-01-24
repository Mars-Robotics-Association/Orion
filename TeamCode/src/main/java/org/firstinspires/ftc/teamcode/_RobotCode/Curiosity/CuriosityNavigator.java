package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.UniversalTurretIntakeArm;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavTuningProfile;
import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavigation;

class CuriosityNavigator extends FreightFrenzyNavigation
{

    public CuriosityNavigator(OpMode setOpMode, MecanumChassis setChassis, UniversalTurretIntakeArm setArm, DuckSpinner setSpinner, DistanceSensor setDuckDist, DistanceSensor setIntakeDist, DistanceSensor setPortDist, DistanceSensor setStarboardDist, ColorSensor setColorSensor, BlinkinController setBlinkin, AllianceSide side) {
        super(setOpMode, setChassis, setArm, setSpinner, setDuckDist, setIntakeDist, setPortDist, setStarboardDist, setColorSensor, setBlinkin, side);

        //CONFIGURATION
        
    }
}
