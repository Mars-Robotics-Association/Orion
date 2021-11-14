package org.firstinspires.ftc.teamcode.WhiteWolf;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

    public WhiteWolfNavigator(HardwareMap setHardwareMap, final OpMode setOpMode){
        opMode = setOpMode;
        hardwareMap = setHardwareMap
    }

    public void Init(){
        mecDrive = new MecanumDrive(new Motor(hardwareMap, "motorOne"));
    }

    public void Update(){

    }

    private MecanumDrive mecDrive;

    HardwareMap hardwareMap;
    OpMode opMode;

}
