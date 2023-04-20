package org.firstinspires.ftc.teamcode._RobotCode.Juan_RELEASED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Camera;

/*
>> import com.acmerobotics.dashboard.FtcDashboard;
>> import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
>> import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
>> import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.Base64Image;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.InputAxis;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
>> import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
>> import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
>> import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
>> import org.firstinspires.ftc.teamcode.Navigation.Camera;
 */

public class Juan_RELEASED extends BaseRobot
{
    public static final String VERSION = "1.16.7";
    public static final JuanPayload_RELEASED.LiftMode LIFT_MODE = JuanPayload_RELEASED.LiftMode.VERSION_1;

    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    JuanPayload_RELEASED payload;
    JuanNavigation_RELEASED navigator;

    //Misc
    FtcDashboard dashboard;
    Camera camera;
    DcMotor lift;
    Servo gripper;

    public static final int x1 = 0;
    public static final int y1 = 0;
    public static final int x2 = 0;
    public static final int y2 = 0;

    static final double liftPower = 2;

    public Juan_RELEASED(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        dashboard = FtcDashboard.getInstance();
        setLog (new HermesLog( "Juan", 200, opMode ));
        setChassisProfile(new _ChassisProfile());
        //camera = new Camera(opMode,"Webcam 1");

        if(USE_CHASSIS) {

            //initialize the chassis & navigator
            navigator = new JuanNavigation_RELEASED(opMode, this);
        }

        if(USE_PAYLOAD){
            lift = opMode.hardwareMap.dcMotor.get("lift");
            gripper = opMode.hardwareMap.servo.get("gripper");
            camera = new Camera(opMode, "Webcam 1");
            lift.setDirection(DcMotor.Direction.REVERSE);
            payload = new JuanPayload_RELEASED(opMode, lift, gripper, liftPower, camera);
        }

        //if(USE_NAVIGATOR){}
    }

    //SETUP METHODS//
    public void init(){

    }

    public void start(){

        getChassis().startChassis();
        getNavigator().setMeasuredPose(0,0,0);
        log.start();
    }

    public void update(){

        if(USE_NAVIGATOR){
            opMode.telemetry.addLine("Updating navigator");
            navigator.update();
            //hermes logging code
            //configures robot code
            RobotPose robotPose = new RobotPose(navigator.getTargetPose()[0],
                    navigator.getTargetPose()[1],Math.toRadians(navigator.getTargetPose()[2]),
                    navigator.getMeasuredPose().getX(), navigator.getMeasuredPose().getY(),navigator.getMeasuredPose().getHeading());
            //converts camera footage to base 64 for gui
            //Base64Image cameraData = new Base64Image(
            //camera.convertBitmapToBase64(camera.shrinkBitmap(camera.getImage(),240,135),0));
            Object[] data = {robotPose};
            log.addData(data);
            log.update();
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
