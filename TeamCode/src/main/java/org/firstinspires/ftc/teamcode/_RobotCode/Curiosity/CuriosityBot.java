package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import android.text.method.Touch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.BasicNavigator;


public class CuriosityBot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    CuriosityExperimentalPayload payload;
    CuriosityNavigator navigator;
    Camera camera;
    ControllerInput gamepad;
    DistanceSensor portDistance;
    DistanceSensor starboardDistance;
    DistanceSensor intakeDistance;
    ColorSensor colorSensor;
    DistanceSensor gripperDist;
    TouchSensor levelSensor;

    //Misc
    FtcDashboard dashboard;

    public CuriosityBot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        dashboard = FtcDashboard.getInstance();
        setLog(new HermesLog("Curiosity", 200, opMode));
        //if(USE_PAYLOAD) camera = new Camera(opMode,"Webcam 1");

        if(USE_CHASSIS) {
            //sensors
            DistanceSensor portDistance = opMode.hardwareMap.get(DistanceSensor.class, "port distance");
            DistanceSensor starboardDistance = opMode.hardwareMap.get(DistanceSensor.class, "starboard distance");
            ColorSensor colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color sensor");

            //initialize the chassis & navigator
            navigator = new BasicNavigator(opMode, this, portDistance, starboardDistance, colorSensor);
        }

        if(USE_PAYLOAD){
            DcMotor liftMotor1 = opMode.hardwareMap.dcMotor.get("Lift 1");
            DcMotor liftMotor2 = opMode.hardwareMap.dcMotor.get("Lift 2");
            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("Arm");
            EncoderActuator arm = new EncoderActuator(opMode, new _ArmProfile(armMotor));
            EncoderActuator lift = new EncoderActuator(opMode, new _LiftProfile(liftMotor1,liftMotor2));
            Servo gripper = opMode.hardwareMap.servo.get("gripper");
            gripperDist = opMode.hardwareMap.get(DistanceSensor.class, "gripper distance");
            levelSensor = opMode.hardwareMap.get(TouchSensor.class, "level sensor");

            payload= new CuriosityExperimentalPayload(opMode, gamepad, lift,
                    arm, gripper, gripperDist, levelSensor);
        }

        if(USE_NAVIGATOR){}
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
            log.Update();
        }
    }

    //make sure to stop everything!
    public void stop(){
        if(USE_CHASSIS){
            navigator.getChassis().stop();
        }
    }


    public BasicNavigator getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}



    public CuriosityNavigator getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public CuriosityExperimentalPayload getPayload(){return payload;}
}
