package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Archive.FieldState.Pose;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;
import org.firstinspires.ftc.teamcode._RobotCode.Erasmus.ErasmusNavigation;
import org.firstinspires.ftc.teamcode._RobotCode.Erasmus._LiftProfile;

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
    public static boolean GRIPPER_AUTOROTATE = true ;

    // Arm
    DcMotorEx armMotor ;
    public static int armTarget = 0 ;
    public static int armHigh = 6376 ;
    public static int armMid = 80 ;
    public static int armLow = 40 ;
    public static int armBottom = 0 ;
    public static double armPower = 1.0 ;
    public static double armKP = 15 ;

    // Lift
    DcMotor leftLiftMotor ;
    DcMotor rightLiftMotor ;
    public static double liftTarget = 0 ;
    public static double liftHigh = 4.2 ;
    public static double liftBottom = 0 ;
    public static double liftPower = 0.7 ;

    // New Lift
    EncoderActuator lift ;

    // Color Sensor
    ColorSensor colorSensor;

    //Misc
    FtcDashboard dashboard;
    HermesLog log;

    // ============================= CONSTRUCTOR ==================================
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
        }

        if(USE_PAYLOAD){
            // ---------- Intake Servo and rotation servo ----------------
            gripperServo = opMode.hardwareMap.servo.get("gripper") ;
            gripperRotate = opMode.hardwareMap.servo.get("gripperRotate") ;
            // --------------------- Arm Control -----------------------
            armMotor = (DcMotorEx) opMode.hardwareMap.dcMotor.get("armMotor");
            //armMotor.setPositionPIDFCoefficients(0.01) ;
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
            armMotor.setTargetPosition(armTarget) ;
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
            armMotor.setPower(armPower) ;
            armKP = armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p ;
            //armMotor.setDirection(DcMotorSimple.Direction.REVERSE) ;
            // ------------------ Lift Control ------------------------
            rightLiftMotor = opMode.hardwareMap.dcMotor.get("rightLiftMotor") ;
            //liftMotor.setDirection(DcMotorSimple.Direction.REVERSE) ;
            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
            leftLiftMotor = opMode.hardwareMap.dcMotor.get("leftLiftMotor") ;
            leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE) ;
            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
            lift = new EncoderActuator(opMode, new _LiftProfile(rightLiftMotor ,leftLiftMotor, 0));
            // ------------------- Color Sensor --------------------
            colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
            colorSensor.enableLed(true);
        }

        if(USE_NAVIGATOR){
            navigator = new ErasmusNavigation(opMode, this, null, null, null);
        }
    }

    // ==================== SETUP METHODS ======================
    public void init(){

    }

    public void start(){
        getChassis().startChassis();
        getNavigator().setMeasuredPose(0,0,0);
        log.start();
    }

    // ========================== UPDATE METHOD ==========================
    public void update(){
        if(USE_NAVIGATOR){
            navigator.update();
        }

        if(USE_PAYLOAD){
            // ------------- Gripper state -----------------
            gripperServo.setPosition(servoTarget) ;
            if (GRIPPER_AUTOROTATE) gripperRotate.setPosition((armMotor.getCurrentPosition()*0.00000957)+0.5) ;
            else gripperRotate.setPosition(gripperRotateTarget) ;
                // ------------- lift state ----------------
            setLiftPosition( liftTarget ) ;
            // ------------- arm state ----------------
            armMotor.setPositionPIDFCoefficients(armKP) ;
            if (armMotor.getTargetPosition() != armTarget) armMotor.setTargetPosition(armTarget) ;
            if (armMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                armMotor.setPower(armPower) ;
            }
        }
        printTelemetry() ;
    }

    // ============ make sure to stop everything! =================
    public void stop(){
        if(USE_CHASSIS){
            navigator.getChassis().stop();
        }
    }

    // =========================== Main telemetry method ==============================
    private void printTelemetry() {
        // ------------ Payload --------------
        opMode.telemetry.addLine("============= DATA ==============");
        opMode.telemetry.addData("Gripper: ", servoTarget);
        opMode.telemetry.addData("Arm:     ", armMotor.getCurrentPosition() + " / " + armTarget ) ;
        opMode.telemetry.addData("Arm P:   ", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p ) ;
        opMode.telemetry.addData("Lift:    ", lift.getPosition(0)+ " / " + liftTarget ) ;
        // ------------ Dead wheel positions ------------
        if (false) {
            opMode.telemetry.addLine("========== Dead wheel positions ===========");
            double[] deadWheelPositions = getNavigator().getDeadWheelPositions();
            opMode.telemetry.addData("LEFT:  ", deadWheelPositions[0]
                    + "  RIGHT: ", deadWheelPositions[1]
                    + "  HORIZ: ", deadWheelPositions[2]);
        }
        // -------------- Odometry estimated pose ------------------
        opMode.telemetry.addLine("============= Robot pose =============");
        Pose2d robotPose = getNavigator().getMeasuredPose();
        opMode.telemetry.addData("X, Y, Angle", Math.round(robotPose.getX()*100)/100
                + ", " + Math.round(robotPose.getY()*100)/100
                + ", " + Math.round(Math.toDegrees(robotPose.getHeading())*100)/100);
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
        openGripper() ;
        waitForTime(waitTime) ;
    }

    public void closeGripper() {
        servoTarget=servoTarget2 ;
        update() ;
    }

    public void closeGripper(double waitTime) {
        closeGripper() ;
        waitForTime(waitTime) ;
    }

    public void setLiftPosition( double newPosition ) {
        lift.goToPosition(newPosition) ;
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
