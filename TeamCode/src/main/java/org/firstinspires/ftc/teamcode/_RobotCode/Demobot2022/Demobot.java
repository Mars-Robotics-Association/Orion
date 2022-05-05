package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Core.OrionObject;
import org.firstinspires.ftc.teamcode.Navigation.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.FreightFrenzy.FreightFrenzyNavigator;

class Demobot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    OrionObject[] components;

    //Misc
    FtcDashboard dashboard;

    public Demobot(OpMode setOpMode, OrionObject[] setComponents) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0));

        components = setComponents;

        opMode = setOpMode;

        dashboard = FtcDashboard.getInstance();
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

    //return whether an object of given name exists in components
    public boolean CheckExistence(String objName){
        for(OrionObject obj : components){
            if(obj.GetName()==objName) return true;
        }
        return false;
    }

    public BlinkinController GetBlinkin(){
        
    }

}
