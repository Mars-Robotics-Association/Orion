package org.firstinspires.ftc.teamcode._RobotCode.Demobot2022;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;


public class Demobot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    DemobotPayload payload;
    DemobotNavigation navigator;

    //Misc
    FtcDashboard dashboard;

    public Demobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        dashboard = FtcDashboard.getInstance();

        if(USE_CHASSIS) {
            //sensors
            DistanceSensor portDistance = opMode.hardwareMap.get(DistanceSensor.class, "port distance");
            DistanceSensor starboardDistance = opMode.hardwareMap.get(DistanceSensor.class, "starboard distance");
            ColorSensor colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color sensor");

            //initialize the chassis & navigator
            navigator = new DemobotNavigation(opMode, this, portDistance, starboardDistance, colorSensor);
        }

        if(USE_PAYLOAD){
            //intake
            DcMotor intakeMotor = opMode.hardwareMap.dcMotor.get("intake");
            MotorArray intake = new MotorArray(new DcMotor[]{intakeMotor},new Servo[]{},new double[]{1},false);
            //path
            DcMotor pathMotor = opMode.hardwareMap.dcMotor.get("path");
            MotorArray path = new MotorArray(new DcMotor[]{pathMotor},new Servo[]{},new double[]{1},false);
            //loader
            Servo loaderMotor = opMode.hardwareMap.servo.get("loader");
            MotorArray loader = new MotorArray(new DcMotor[]{},new Servo[]{loaderMotor},new double[]{1},false);
            //turret
            DcMotor turretMotor = opMode.hardwareMap.dcMotor.get("turret");
            EncoderActuator turret = new EncoderActuator(opMode, new _TurretProfile(turretMotor));
            //shooter
            DcMotor shooterMotor = opMode.hardwareMap.dcMotor.get("shooter");
            MotorArray shooter = new MotorArray(new DcMotor[]{shooterMotor},new Servo[]{},new double[]{1},false);
            //initialize payload
            payload = new DemobotPayload(opMode,intake,path,loader,turret,shooter);
        }

        if(USE_NAVIGATOR){}
    }

    //SETUP METHODS//
    public void init(){

    }

    public void start(){
        getChassis().startChassis();
        getNavigator().setRobotPose(0,0,0);
    }

    public void update(){

        if(USE_NAVIGATOR){
            navigator.update();
        }
    }

    //make sure to stop everything!
    public void stop(){
        if(USE_CHASSIS){
            navigator.getChassis().stop();
        }
    }


    public DemobotNavigation getNavigator(){return navigator;}
    public MecanumChassis getChassis(){return navigator.getChassis();}
    public DemobotPayload getPayload(){return payload;}


}
