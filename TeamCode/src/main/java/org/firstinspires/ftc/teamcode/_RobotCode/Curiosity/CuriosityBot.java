package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.Base64Image;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Camera;


public class CuriosityBot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    HermesLog hermesLog;
    //Mechanical Components
    CuriosityNavigator navigator;
    Camera camera;

    //Misc
    FtcDashboard dashboard;

    public CuriosityBot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        dashboard = FtcDashboard.getInstance();
        hermesLog = new HermesLog("Curiosity", 200, opMode);
        camera = new Camera(opMode,"Webcam 1");

        if(USE_CHASSIS) {
            //sensors
            DistanceSensor portDistance = opMode.hardwareMap.get(DistanceSensor.class, "port distance");
            DistanceSensor starboardDistance = opMode.hardwareMap.get(DistanceSensor.class, "starboard distance");
            ColorSensor colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color sensor");

            //initialize the chassis & navigator
            setChassisProfile(new _ChassisProfile());
            navigator = new CuriosityNavigator(opMode, this, portDistance, starboardDistance, colorSensor);
        }

        if(USE_PAYLOAD){

        }

        if(USE_NAVIGATOR){}
    }

    //SETUP METHODS//
    public void init(){

    }

    public void start(){
        getChassis().startChassis();
        getNavigator().setMeasuredPose(0,0,0);
        hermesLog.start();
    }

    public void update() throws InterruptedException {
        if(USE_CHASSIS){
        }
        if(USE_NAVIGATOR){
            navigator.update();
            //hermes logging code
            //configures robot code
            RobotPose robotPose = new RobotPose(navigator.getTargetPose().getX(),
                    navigator.getTargetPose().getY(),navigator.getTargetPose().getHeading(),
                    navigator.getMeasuredPose().getX(), navigator.getMeasuredPose().getY(),navigator.getMeasuredPose().getHeading());
            //converts camera footage to base 64 for gui
            Base64Image cameraData = new Base64Image(
                    camera.convertBitmapToBase64(camera.shrinkBitmap(camera.getImage(),480,270),100));
            Object[] data = {robotPose, cameraData};
            hermesLog.addData(data);
            hermesLog.Update();
        }
    }

    //make sure to stop everything!
    public void stop(){
        if(USE_CHASSIS){
            navigator.getChassis().stop();
        }
    }


    public CuriosityNavigator getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}


}
