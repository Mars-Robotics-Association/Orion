package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;

public class Juan extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    JuanPayload payload;
    JuanNavigation navigator;

    //Misc
    FtcDashboard dashboard;

    public Juan(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        dashboard = FtcDashboard.getInstance();

        if(USE_CHASSIS) {

            //initialize the chassis & navigator
            navigator = new JuanNavigation(opMode, this);
        }

        if(USE_PAYLOAD){
            payload = new JuanPayload(opMode);
        }

        if(USE_NAVIGATOR){}
    }

    //SETUP METHODS//
    public void init(){

    }

    public void start(){
        getChassis().startChassis();
        getNavigator().setRobotPose(0,0,0);
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


    public JuanNavigation getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public JuanPayload getPayload(){return payload;}


}
