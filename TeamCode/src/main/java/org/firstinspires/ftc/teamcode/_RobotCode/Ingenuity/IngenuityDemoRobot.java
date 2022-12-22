package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.Base64Image;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Camera;

@Config
public class IngenuityDemoRobot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;

    //Mechanical Components
    IngenuityPayload payload;
    IngenuityNavigation navigator;
    Camera camera;
    HermesLog hermesLog;

    // Gripper
    Servo gripperServo ;
    public static double servoTarget=0.5 ;
    public static double servoTarget1=0.4 ;
    public static double servoTarget2=0.71 ;

    // Gripper rotation
    Servo gripperRotate ;
    public static double gripperRotateTarget = 0.5 ;
    public static double gripperRotateHigh = 0.61 ;
    public static double gripperRotateLow= 0.50 ;

    // Arm
    DcMotor armMotor ;
    public static int armTarget = 0 ;
    public static int armHigh = 5302 ;
    public static int armLow = 0 ;

    //Misc
    FtcDashboard dashboard;

    public IngenuityDemoRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        hermesLog = new HermesLog("Ingenuity", 200, opMode);
        dashboard = FtcDashboard.getInstance();
        camera = new Camera(opMode,"Webcam 1");

        if(USE_CHASSIS) {
            //sensors
            //DistanceSensor portDistance = opMode.hardwareMap.get(DistanceSensor.class, "port distance");
            //DistanceSensor starboardDistance = opMode.hardwareMap.get(DistanceSensor.class, "starboard distance");
            //ColorSensor colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color sensor");

            //initialize the chassis & navigator
            setChassisProfile(new _ChassisProfile());
            navigator = new IngenuityNavigation(opMode, this, null, null, null);
        }

        if(USE_PAYLOAD){
            // Intake
            gripperServo = opMode.hardwareMap.servo.get("gripper") ;
            gripperRotate = opMode.hardwareMap.servo.get("gripperRotate") ;
            // Arm Control
            armMotor = opMode.hardwareMap.dcMotor.get("armMotor") ;
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE) ;
            //payload = new IngenuityPayload(opMode); // Maybe refactor payload to its own class later
        }

        if(USE_NAVIGATOR){

        }
    }

    //SETUP METHODS//
    public void init(){

    }

    public void start(){
        getChassis().startChassis();
        getNavigator().setMeasuredPose(0,0,0);
        hermesLog.start();
    }

    public void update(){

        if(USE_NAVIGATOR){
            navigator.update();
            //hermes logging code
            //configures robot code
            RobotPose robotPose = new RobotPose(navigator.getTargetPose().getX(),
                    navigator.getTargetPose().getY(),navigator.getTargetPose().getHeading(),
                    navigator.getMeasuredPose().getX(), navigator.getMeasuredPose().getY(),navigator.getMeasuredPose().getHeading());
            //converts camera footage to base 64 for gui
            Base64Image cameraData = null;
            try {
                cameraData = new Base64Image(
                        camera.convertBitmapToBase64(camera.shrinkBitmap(camera.getImage(),480,270),10));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Object[] data = {robotPose, cameraData};
            hermesLog.addData(data);
            hermesLog.Update();
        }

        if(USE_PAYLOAD){
            gripperServo.setPosition(servoTarget) ;
            //gripperRotate.setPosition(gripperRotateTarget) ;
            gripperRotate.setPosition((armMotor.getCurrentPosition()*0.00002075)+0.5) ;
            armMotor.setTargetPosition(armTarget) ;
            if (armMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                armMotor.setPower(0.7) ;
            }
        }
    }

    //make sure to stop everything!
    public void stop(){
        if(USE_CHASSIS){
            navigator.getChassis().stop();
        }
    }

    public void toggleGripper() {
        if(servoTarget==servoTarget1){
            servoTarget=servoTarget2 ;
        }
        else servoTarget=servoTarget1 ;
    }

    public void openGripper() {
        servoTarget=servoTarget1 ;
        update() ;
    }

    public void closeGripper() {
        servoTarget=servoTarget2 ;
        update() ;
    }

    //Wait for a period of time (seconds)
    public void waitForTime(double time) {
        double startTime = opMode.getRuntime();
        while (IsTimeUp(startTime,time)){
            update() ;
        }
    }

    public boolean IsTimeUp(double startTime, double runTime) { return opMode.getRuntime()<startTime+runTime ; } // From Owen

    public IngenuityNavigation getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public IngenuityPayload getPayload(){return payload;}
}
