package org.firstinspires.ftc.teamcode._RobotCode.Oppertunity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.BaseRobots.MecanumBaseControl;

/**
 * Control class for the Kenobi Robot. Controls payload.
 * Required to run: Phones | REV Hub | Belinda Chassis
 * Suggested to run: Shooter | Intake | Odometry | Webcam
 */
//The class used to control the demobot. Autonomous functions, opmodes, and other scripts can call
//methods in here to control the demobot.

//REQUIRED TO RUN: Phones | REV Hub | Demobot Chassis | Shooter | Odometry Unit

public class OpportunityUltimateGoalControl extends MecanumBaseControl
{
    ////Dependencies////

    ////Variables////


    /**@param setOpMode pass the opmode running this down to access hardware map
     * @param useChassis whether to use the chassis of the robot
     * @param usePayload whether to use the shooter/intake/lift of the robot
     * @param useNavigator whether to use Orion (webcams + odometry navigation)
     */
    public OpportunityUltimateGoalControl(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, useChassis, usePayload, useNavigator);
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===
        if(USE_PAYLOAD) {
            //Robot Hardware goes here
        }

        //TODO ===INIT CORE ROBOT===
        super.InitCoreRobotModules();

        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){
        super.StartCoreRobotModules();
    }
}
