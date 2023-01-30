package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Camera;

public class Juan extends BaseRobot
{
    public static final String VERSION = "1.17";
    public static final SleeveReader.POST_PROCESS POST_PROCESS = SleeveReader.POST_PROCESS.BEAUTIFUL;
    public static final JuanPayload.LiftMode LIFT_MODE = JuanPayload.LiftMode.VERSION_1;

    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    JuanPayload payload;
    JuanNavigation navigator;

    //Misc
    FtcDashboard dashboard;
    Camera camera;

    private void t(int num) throws InterruptedException {
        opMode.telemetry.addData("line", num);
        opMode.telemetry.update();
        sleep(1000);
    }

    static final double liftPower = 2;

    public Juan(OpMode opMode, boolean useChassis, boolean usePayload, boolean useNavigator) {

        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode.telemetry.addLine("JUAN");
        opMode.telemetry.update();

        try {
        t(0);this.opMode = opMode;

        t(1);dashboard = FtcDashboard.getInstance();

        t(2);if(USE_CHASSIS) {

        t(3);    //initialize the chassis & navigator
        t(4);    navigator = new JuanNavigation(opMode, this);
        t(5);}

        t(6);if(USE_PAYLOAD){
        t(7);    DcMotor lift = opMode.hardwareMap.dcMotor.get("lift");
        t(8);    Servo gripper = opMode.hardwareMap.servo.get("gripper");
        t(9);    Camera camera = new Camera(opMode, "Webcam 1");
        t(10);    payload = new JuanPayload(opMode, false, lift, gripper, liftPower, camera);
        t(11);}
        }catch(InterruptedException e){
            e.printStackTrace();
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
