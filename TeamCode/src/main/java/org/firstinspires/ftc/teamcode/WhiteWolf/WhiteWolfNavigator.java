package org.firstinspires.ftc.teamcode.WhiteWolf;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
    ----WARNING-----
    This utility is in early development and shouldn't be used at competition.
    In the meantime, consider using Orion.

    STATUS: UNUSABLE
*/

////SENSING////
// Gamepad

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

    public WhiteWolfNavigator(HardwareMap setHardwareMap, String setOpType){
        hardwareMap = setHardwareMap;
        opType = setOpType;
    }

    private final HardwareMap hardwareMap;
    private Motor fL, fR, bL, bR;
    private MecanumDrive mecDrive;
    private GamepadEx gamepad;
    private String opType;

    public void init(){
        fL = new Motor(hardwareMap, "FL");
        fR = new Motor(hardwareMap, "FR");
        bL = new Motor(hardwareMap, "RR");
        bR = new Motor(hardwareMap, "RL");

        mecDrive = new MecanumDrive(fL, fR, bL, bR);
    }

    public void loop() {
        if(opType.equals("TeleOp")) {
            mecDrive.driveRobotCentric(
                    gamepad.getLeftX(),
                    gamepad.getLeftY(),
                    gamepad.getRightY()
            );
        }
    }
}
