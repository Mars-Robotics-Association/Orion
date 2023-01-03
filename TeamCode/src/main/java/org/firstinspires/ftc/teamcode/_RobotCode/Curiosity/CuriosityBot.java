package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.Base64Image;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.InputSystem.InputAxis;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Camera;


public class CuriosityBot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    CuriosityPayload payload;
    CuriosityNavigator navigator;
    Camera camera;
    ControllerInput gamepad;

    //Misc
    FtcDashboard dashboard;

    public CuriosityBot(OpMode setOpMode, ControllerInput setGamepad, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        gamepad = setGamepad;
        dashboard = FtcDashboard.getInstance();
        setLog(new HermesLog("Curiosity", 200, opMode));
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
            DcMotor armMotor1 = opMode.hardwareMap.dcMotor.get("Arm 1");
            DcMotor armMotor2 = opMode.hardwareMap.dcMotor.get("Arm 2");
            EncoderActuator arm = new EncoderActuator(opMode, new _ArmProfile(armMotor1,armMotor2));
            Servo gripper = opMode.hardwareMap.servo.get("gripper");
            DistanceSensor gripperDist = opMode.hardwareMap.get(DistanceSensor.class, "gripper distance");
            DistanceSensor armLevelDist = opMode.hardwareMap.get(DistanceSensor.class, "arm level distance");;

            payload= new CuriosityPayload(opMode, gamepad, new Triggers_InputAxis(gamepad), new Bumpers_InputAxis(gamepad),
                    arm, gripper, gripperDist, armLevelDist);
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
            //Base64Image cameraData = new Base64Image(
                    //camera.convertBitmapToBase64(camera.shrinkBitmap(camera.getImage(),240,135),0));
            Object[] data = {robotPose};
            log.addData(data);
            log.Update();
        }
        if(USE_PAYLOAD){
            payload.update();
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
    public CuriosityPayload getPayload(){return payload;}
}


//INPUT AXIS
class Triggers_InputAxis implements InputAxis {
    ControllerInput gamepad;
    public Triggers_InputAxis(ControllerInput setGamepad){gamepad = setGamepad;}
    @Override
    public double getValue() {
        if(gamepad.getRT()) return 1;
        else if(gamepad.getLT()) return -1;
        else return 0;
    }
}

class Bumpers_InputAxis implements InputAxis {
    ControllerInput gamepad;
    public Bumpers_InputAxis(ControllerInput setGamepad){gamepad = setGamepad;}
    @Override
    public double getValue() {
        if(gamepad.getRB()) return 1;
        else if(gamepad.getLB()) return -1;
        else return 0;
    }
}
