package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.opencv.core.Rect;

public class Juan extends BaseRobot
{
    public static final String VERSION = "1.17";
    public static final JuanPayload.LiftMode LIFT_MODE = JuanPayload.LiftMode.VERSION_1;
    public static final Rect SCAN_BOUNDS = new Rect(
            640, 360, 640, 360
    );

    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    JuanPayload payload;
    JuanNavigation navigator;

    //Misc
    FtcDashboard dashboard;

    static final double liftPower = 2;

    public Juan(OpMode opMode, boolean useChassis, boolean usePayload, boolean useNavigator) {

        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode.telemetry.addLine("JUAN");
        opMode.telemetry.update();

        this.opMode = opMode;

        dashboard = FtcDashboard.getInstance();

        if (USE_CHASSIS) {

            //initialize the chassis & navigator
            navigator = new JuanNavigation(opMode, this);
        }

        if (USE_PAYLOAD) {
            payload = new JuanPayload(opMode, opMode.hardwareMap);
        }

        //if(USE_NAVIGATOR){}
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
//        if(USE_CHASSIS){
//            navigator.getChassis().stop();
//        }

    }

    public JuanNavigation getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public JuanPayload getPayload(){return payload;}
}
