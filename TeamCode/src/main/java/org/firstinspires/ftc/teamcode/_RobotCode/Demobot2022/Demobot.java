package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;

class Demobot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    BlinkinController blinkinController;
    DemobotPayload payload;
    DemobotNavigation navigator;

    //Misc
    FtcDashboard dashboard;

    public Demobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        dashboard = FtcDashboard.getInstance();

        if(USE_CHASSIS) {
            //initialize the chassis
            navigator = new DemobotNavigation(opMode, this);
        }

        if(USE_PAYLOAD){
            //motors

            //sensors

            blinkinController = new BlinkinController(opMode);

        }

        if(USE_NAVIGATOR){
            //sensors

            //initialize navigator

        }
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===

        //TODO ===INIT CORE ROBOT===
        getChassis().InitCoreRobotModules();


        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){
        getChassis().StartCoreRobotModules();
        //if(USE_NAVIGATOR) navigator.NavigatorOn();
    }

    public void Update(){
        if(USE_PAYLOAD){
        }
    }

    //TODO make sure to stop everything
    public void Stop(){
        if(USE_PAYLOAD) {

        }
    }


    public BlinkinController getLights(){return blinkinController;}
    public DemobotNavigation getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public DemobotPayload getPayload(){return payload;}


}
