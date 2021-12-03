package org.firstinspires.ftc.teamcode._RobotCode.SampleBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._RobotCode.Archived.MecanumBaseControl;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode._RobotCode._Defaults.DefaultNavProfile;

//REQUIRED TO RUN: Phones | REV Hub | Chassis

@Config
public class SampleBot extends MecanumBaseControl
{
    ////Dependencies////
    //Mechanical Components

    ////Variables////
    //Calibration
    private double levelPitchThreshold = 5;


    /**@param setOpMode pass the opmode running this down to access hardware map
     * @param useChassis whether to use the chassis of the robot
     * @param usePayload whether to use the shooter/intake/lift of the robot
     * @param useNavigator whether to use Orion (webcams + odometry navigation)
     */
    public SampleBot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new DefaultNavProfile(), new HermesLog("SAMPLE", 500, setOpMode), useChassis, usePayload, useNavigator);
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===

        //TODO ===INIT CORE ROBOT===
        super.InitCoreRobotModules();


        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){
        super.StartCoreRobotModules();
    }

}
