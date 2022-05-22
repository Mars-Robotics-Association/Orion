package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Extras.BlinkinController;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FreightFrenzy.FreightFrenzyNavigator;

class ErasmusRobot extends BaseRobot
{
    ////Dependencies////
    OpMode opMode;
    //Mechanical Components
    public MecanumChassis chassis;
    public ErasmusTurretArm turretArm;
    public FreightFrenzyNavigator navigator;
    DuckSpinner duckSpinner;
    BlinkinController blinkinController;

    //Misc
    FtcDashboard dashboard;

    public ErasmusRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        //set up robot state parent
        super(FieldSide.BLUE,new Pose(0,0,0),usePayload,useChassis,useNavigator);
        opMode = setOpMode;

        dashboard = FtcDashboard.getInstance();

        if(USE_CHASSIS) {
            //initialize the chassis
            chassis = new MecanumChassis(setOpMode, new _ChassisProfile(), new HermesLog("Erasmus", 200, setOpMode), this);
        }

        if(USE_PAYLOAD){
            //motors
            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("Arm");
            //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            DcMotor turretMotor = opMode.hardwareMap.dcMotor.get("Turret");
            DcMotor duckMotor = opMode.hardwareMap.dcMotor.get("Duck");
            Servo intakeMotor = opMode.hardwareMap.servo.get("Intake");

            //sensors
            DistanceSensor intakeDist = opMode.hardwareMap.get(DistanceSensor.class, "intakeDist");
            DistanceSensor duckDist = opMode.hardwareMap.get(DistanceSensor.class, "duckDist");
            DistanceSensor armLevelDist = opMode.hardwareMap.get(DistanceSensor.class, "armResetDist");


            blinkinController = new BlinkinController(opMode);

            turretArm = new ErasmusTurretArm(opMode, this, blinkinController, new _ArmProfile(armMotor), new _TurretProfile(turretMotor),
                    intakeMotor, intakeDist, armLevelDist,false);
            turretArm.arm.ResetToZero();

            duckSpinner = new DuckSpinner(duckMotor, 1);

        }

        if(USE_NAVIGATOR){
            //sensors
            DistanceSensor intakeDist = opMode.hardwareMap.get(DistanceSensor.class, "intakeDist");
            DistanceSensor duckDist = opMode.hardwareMap.get(DistanceSensor.class, "duckDist");
            DistanceSensor portDist = opMode.hardwareMap.get(DistanceSensor.class, "portDist");
            DistanceSensor starboardDist = opMode.hardwareMap.get(DistanceSensor.class, "starboardDist");
            DistanceSensor armLevelDist = opMode.hardwareMap.get(DistanceSensor.class, "armResetDist");
            ColorSensor colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
            //initialize navigator
            navigator = new FreightFrenzyNavigator(opMode, this, chassis, turretArm, duckSpinner,
                    duckDist, intakeDist, armLevelDist, portDist, starboardDist, colorSensor, blinkinController);
        }
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===

        //TODO ===INIT CORE ROBOT===
        chassis.InitCoreRobotModules();


        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){

        chassis.StartCoreRobotModules();
        if(USE_NAVIGATOR) navigator.NavigatorOn();
    }

    public void Update(){
        if(USE_PAYLOAD){
        }
    }

    public void Stop(){
        //navigation.StopNavigator();
        if(USE_PAYLOAD) {
            turretArm.StopArmThread();
            turretArm.armThread.StopAutoIntake();
            turretArm.armThread.StopAutoLevelling();
            turretArm.StopIntake();
            turretArm.arm.SetPowerRaw(0);
        }
    }

    public ErasmusTurretArm TurretArm(){return turretArm;}
    public EncoderActuator Turret(){return turretArm.turret;}
    public EncoderActuator Arm(){return turretArm.arm;}
    public FreightFrenzyNavigator Navigator(){return navigator;}

    public DuckSpinner DuckSpinner(){return duckSpinner;}

    public BlinkinController Lights(){return blinkinController;}

}
