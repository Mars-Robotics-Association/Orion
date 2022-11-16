package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;

@Config
public class IngenuityPowerPlayBot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    IngenuityPayload payload;
    IngenuityNavigation navigator;
    Servo gripperServo ;
    public static double servoTarget=0.8;
    public static double servoTarget1=0.8;
    public static double servoTarget2=0.8;

    //Misc
    FtcDashboard dashboard;
    HermesLog log;

    public IngenuityPowerPlayBot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
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
            // DcMotor intakeMotor = opMode.hardwareMap.dcMotor.get("Intake");
            // Servo loaderServo = opMode.hardwareMap.servo.get("Loader");
            gripperServo= opMode.hardwareMap.servo.get("gripper");
            payload = new IngenuityPayload(opMode);
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
        if(IngenuityPowerPlayBot.servoTarget==servoTarget1){
            IngenuityPowerPlayBot.servoTarget=servoTarget2;
        }
        else IngenuityPowerPlayBot.servoTarget=servoTarget1 ;
    }


    public IngenuityNavigation getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public IngenuityPayload getPayload(){return payload;}
}
