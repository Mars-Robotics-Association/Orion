package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode._RobotCode.Erasmus.ErasmusNavigation;

@Config
public class ErasmusRobot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;

    //Mechanical Components
    ErasmusNavigation navigator;

    // Gripper
    Servo gripperServo ;
    public static double servoTarget=0.5 ;
    public static double servoTarget1=0.4 ;
    public static double servoTarget2=0.73 ;

    // Gripper rotation
    Servo gripperRotate ;
    public static double gripperRotateTarget = 0.5 ;
    public static double gripperRotateHigh = 0.61 ;
    public static double gripperRotateLow= 0.50 ;

    // Arm
    DcMotor armMotor ;
    public static int armTarget = 0 ;
    public static int armHigh = 5302 ;
    public static int armMid = 2850 ;
    public static int armLow = 1875 ;
    public static int armBottom = 0 ;
    public static int armAuto = 800 ;

    DcMotor liftMotor ;

    // Color Sensor
    ColorSensor colorSensor;


    //Misc
    FtcDashboard dashboard;
    HermesLog log;

    public ErasmusRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        log = new HermesLog("Erasmus", 50, opMode);
        dashboard = FtcDashboard.getInstance();

        if(USE_CHASSIS) {
            //sensors
            //DistanceSensor portDistance = opMode.hardwareMap.get(DistanceSensor.class, "port distance");
            //DistanceSensor starboardDistance = opMode.hardwareMap.get(DistanceSensor.class, "starboard distance");
            //ColorSensor colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color sensor");

            //initialize the chassis & navigator
            setChassisProfile(new org.firstinspires.ftc.teamcode._RobotCode.Erasmus._ChassisProfile());
            navigator = new ErasmusNavigation(opMode, this, null, null, null);
        }

        if(USE_PAYLOAD){
            // Intake Servo and rotation servo
            gripperServo = opMode.hardwareMap.servo.get("gripper") ;
            gripperRotate = opMode.hardwareMap.servo.get("gripperRotate") ;
            // Arm Control
            armMotor = opMode.hardwareMap.dcMotor.get("armMotor") ;
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE) ;
            // Lift Control
            liftMotor = opMode.hardwareMap.dcMotor.get("liftMotor") ;
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER) ;
            //armMotor.setDirection(DcMotorSimple.Direction.REVERSE) ;
            // Color Sensor
            colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
            colorSensor.enableLed(true);
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
        log.start();
    }

    public void update(){

        if(USE_NAVIGATOR){
            navigator.update();
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

    public void openGripper(double waitTime) {
        servoTarget=servoTarget1 ;
        update() ;
        waitForTime(waitTime) ;
    }

    public void closeGripper() {
        servoTarget=servoTarget2 ;
        update() ;
    }

    public void closeGripper(double waitTime) {
        servoTarget=servoTarget2 ;
        update() ;
        waitForTime(waitTime) ;
    }

    public int checkSignal() {
        int result = 2 ;
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
        if (hsvValues[0]< 80) { result = 1 ; }
        else if (hsvValues[0] > 190) { result = 3 ; }
        return result ;
    }

    //Wait for a period of time (seconds)
    public void waitForTime(double time) {
        double startTime = opMode.getRuntime();
        while (IsTimeUp(startTime,time)){
            update() ;
        }
    }

    public boolean IsTimeUp(double startTime, double runTime) { return opMode.getRuntime()<startTime+runTime ; } // From Owen

    public ErasmusNavigation getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
}
