package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Camera;

@Config
public class IngenuityPowerPlayBot extends BaseRobot
{
    public enum SignalColor {
        BLUE,
        RED,
        GREEN
    }

    ////Dependencies////
    OpMode opMode;
    HermesLog log;

    //Mechanical Components
    IngenuityPayload payload;
    IngenuityNavigation navigator;

    // Gripper
    Servo gripperServo ;
    public static double servoTarget=0.5;
    public static double servoTarget1=0.37;//closed
    public static double servoTarget2=0.7;//open
    ColorSensor colorSensor;
    DistanceSensor sensorDistance ;

    //Misc
    FtcDashboard dashboard;

    public IngenuityPowerPlayBot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator, double armPos) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        log = new HermesLog("Ingenuity", 50, opMode);
        dashboard = FtcDashboard.getInstance();

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
            //intake
            gripperServo= opMode.hardwareMap.servo.get("gripper");
            colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
            sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "colorSensor");

            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("armMotor") ;
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE) ;
            payload = new IngenuityPayload(opMode, armMotor, armPos);
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
            RobotPose robotPose = new RobotPose(navigator.getTargetPose()[0],
                    navigator.getTargetPose()[1],navigator.getTargetPose()[2],
                    navigator.getMeasuredPose().getX(), navigator.getMeasuredPose().getY(),navigator.getMeasuredPose().getHeading());
            //converts camera footage to base 64 for gui
            //Base64Image cameraData = new Base64Image(
            //camera.convertBitmapToBase64(camera.shrinkBitmap(camera.getImage(),240,135),0));
            Object[] data = {robotPose};
            log.addData(data);
            log.Update();
        }

        if(USE_PAYLOAD){
            gripperServo.setPosition(servoTarget);

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
            servoTarget=servoTarget2;
        }
        else servoTarget=servoTarget1 ;
    }

    public SignalColor readSignal() {
        SignalColor result = SignalColor.GREEN;
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
        opMode.telemetry.addData("hsv ", hsvValues[0]);
        if (hsvValues[0] < 90) result = SignalColor.RED;
        else if (hsvValues[0] > 300)
            result = SignalColor.RED;//red wraps back around: it's centered on 0/360 degrees
        else if (hsvValues[0] > 190) result = SignalColor.BLUE;
        opMode.telemetry.addData("result", result);
        return result;
    }


    public IngenuityNavigation getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public IngenuityPayload getPayload(){return payload;}
}
