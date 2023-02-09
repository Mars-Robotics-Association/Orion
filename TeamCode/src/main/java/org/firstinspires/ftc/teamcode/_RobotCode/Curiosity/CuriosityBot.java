package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;
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
    DistanceSensor portDistance;
    DistanceSensor starboardDistance;
    DistanceSensor intakeDistance;
    ColorSensor colorSensor;
    DistanceSensor gripperDist;
    TouchSensor levelSensor;

    //Misc
    FtcDashboard dashboard;

    public CuriosityBot(OpMode setOpMode, ControllerInput setGamepad, boolean useChassis, boolean usePayload, boolean useNavigator, boolean useCamera) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        gamepad = setGamepad;
        dashboard = FtcDashboard.getInstance();
        setLog(new HermesLog("Curiosity", 200, opMode));

        if(USE_PAYLOAD) camera = new Camera(opMode,"Webcam 1");



        if(USE_CHASSIS) {
            //sensors
            portDistance = opMode.hardwareMap.get(DistanceSensor.class, "port distance");
            starboardDistance = opMode.hardwareMap.get(DistanceSensor.class, "starboard distance");
            intakeDistance = opMode.hardwareMap.get(DistanceSensor.class, "intake distance");
            colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color sensor");

            //initialize the chassis & navigator
            setChassisProfile(new _ChassisProfile());
            navigator = new CuriosityNavigator(opMode, this, portDistance, starboardDistance, colorSensor);
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

            payload= new CuriosityPayload(opMode, gamepad, lift,
                    arm, gripper, gripperDist, levelSensor, new BlinkinController(opMode));

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
        if(USE_PAYLOAD){
        }
    }

    //make sure to stop everything!
    public void stop(){
        if(USE_CHASSIS){
            navigator.getChassis().stop();
        }
    }

    //goes to cone or cone stack and picks up cone then backs up

    //line follows until detects something in guide area

    //drives straight forwards with user correction until detects something in guide area
    public boolean driveToCone(double speed){
        opMode.telemetry.addLine("DRIVING TO CONE");
        opMode.telemetry.addData("Distance to cone", intakeDistance.getDistance(DistanceUnit.CM));
        //drive if robot isn't there and stop when its gets there
        if(intakeDistance.getDistance(DistanceUnit.CM)>4) {
            //navigator.rawDriveWithControllerOffsets(gamepad,.2,180,.4,0);
            double turnOffset = navigator.calculateControllerInputOffsets(gamepad,0.2)[2];
            getChassis().rawDrive(180,0.4,turnOffset);
            return true;
        }
        else{
            getChassis().stop();
            return false;
        }
    }


    public CuriosityNavigator getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public CuriosityPayload getPayload(){return payload;}
}


//INPUT AXIS
//class Triggers_InputAxis implements InputAxis {
//    ControllerInput gamepad;
//    public Triggers_InputAxis(ControllerInput setGamepad){gamepad = setGamepad;}
//    @Override
//    public double getValue() {
//        if(gamepad.getRT()) return 1;
//        else if(gamepad.getLT()) return -1;
//        else return 0;
//    }
//}
//
//class Bumpers_InputAxis implements InputAxis {
//    ControllerInput gamepad;
//    public Bumpers_InputAxis(ControllerInput setGamepad){gamepad = setGamepad;}
//    @Override
//    public double getValue() {
//        if(gamepad.getRB()) return 1;
//        else if(gamepad.getLB()) return -1;
//        else return 0;
//    }
//}
