package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;


public class Demobot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
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
            //initialize the chassis & navigator
            navigator = new DemobotNavigation(opMode, this);
        }

        if(USE_PAYLOAD){}

        if(USE_NAVIGATOR){}
    }

    //SETUP METHODS//
    public void init(){

    }

    public void start(){
        getChassis().startChassis();
    }

    public void update(){

        if(USE_NAVIGATOR){
            navigator.update();
        }
    }

    //make sure to stop everything!
    public void stop(){
        if(USE_CHASSIS){
            navigator.getChassis().stop();
        }
    }


    public DemobotNavigation getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public DemobotPayload getPayload(){return payload;}


}
