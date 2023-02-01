package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Camera;

public class Juan_RELEASED extends BaseRobot
{
    public static final String VERSION = "1.16.5";
    public static final JuanPayload_RELEASED.LiftMode LIFT_MODE = JuanPayload_RELEASED.LiftMode.VERSION_1;

    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    JuanPayload_RELEASED payload;
    JuanNavigation_RELEASED navigator;

    //Misc
    FtcDashboard dashboard;
    Camera camera;

    public static final int x1 = 0;
    public static final int y1 = 0;
    public static final int x2 = 0;
    public static final int y2 = 0;

    static final double liftPower = 2;

    public Juan_RELEASED(OpMode opMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        this.opMode = opMode;

        dashboard = FtcDashboard.getInstance();

        if(USE_CHASSIS) {

            //initialize the chassis & navigator
            navigator = new JuanNavigation_RELEASED(opMode, this);
        }

        if(USE_PAYLOAD){
            DcMotor lift = opMode.hardwareMap.dcMotor.get("lift");
            Servo gripper = opMode.hardwareMap.servo.get("gripper");
            Camera camera = new Camera(opMode, "Webcam 1");
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            payload = new JuanPayload_RELEASED(opMode, lift, gripper, liftPower, camera);
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
        if(USE_CHASSIS){
            navigator.getChassis().stop();
        }
    }

    public JuanNavigation_RELEASED getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public JuanPayload_RELEASED getPayload(){return payload;}
}