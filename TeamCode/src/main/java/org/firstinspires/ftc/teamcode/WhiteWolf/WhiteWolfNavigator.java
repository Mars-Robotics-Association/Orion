package org.firstinspires.ftc.teamcode.WhiteWolf;

import com.arcrobotics.ftclib.drivebase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumBaseControl;

/*
    ----WARNING-----
    This utility is in early development and shouldn't be used at competition.
    In the meantime, consider using Orion.

    STATUS: UNUSABLE
*/

////SENSING////
// GamePad

////DRIVING////
// Pure Pursuit

////COLLISION AVOIDANCE////
// A*

public class WhiteWolfNavigator
{

    /*
        public WhiteWolfNavigator(OpMode setOpMode, DifferentialDrive diffDrive){
            opMode = setOpMode;
            control = setControl;
            navigationProfile = setNavProfile;
        }
        // For Differential and Tank bases
    */

    public WhiteWolfNavigator(OpMode setOpMode){
        opMode = setOpMode;

    }

    public void Init(){
        mecDrive = new MecanumDrive(opMode.hardwareMap.dcMotor.get("fl"), opMode.hardwareMap.dcMotor.get("fr"),
                opMode.hardwareMap.dcMotor.get("bl"), opMode.hardwareMap.dcMotor.get("br"));
    }

    public void Update(){

    }

    private OpMode opMode;
    private MecanumDrive mecDrive;

}
