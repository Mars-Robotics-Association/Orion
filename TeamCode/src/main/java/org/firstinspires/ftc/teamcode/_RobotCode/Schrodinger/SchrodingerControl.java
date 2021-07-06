package org.firstinspires.ftc.teamcode._RobotCode.Schrodinger;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.BaseRobots.MecanumBaseControl;
import org.firstinspires.ftc.teamcode.MechanicalControl.MotorizedIntake;
import org.firstinspires.ftc.teamcode._RobotCode.Schrodinger.MechanicalControllers.SchrodingerArm;
import org.firstinspires.ftc.teamcode._RobotCode.Schrodinger.MechanicalControllers.SchrodingerFoundationGrabbers;
import org.firstinspires.ftc.teamcode._RobotCode.Schrodinger.MechanicalControllers.SchrodingerGripper;
import org.firstinspires.ftc.teamcode.Orion.Odometry.DemoBotOdometry;

//The class used to control schrodinger. Autonomous functions, opmodes, and other scripts can call
//methods in here to control the schrodinger.

//REQUIRED TO RUN: Phones | REV Hub | Demobot Chassis | Shooter | Odometry Unit

public class SchrodingerControl extends MecanumBaseControl
{
    ////Dependencies////
    //Mechanical Components
    private SchrodingerArm arm;
    private SchrodingerGripper gripper;
    private MotorizedIntake intake;
    private SchrodingerFoundationGrabbers grabbers;
    private DemoBotOdometry odometry;

    //Sensors
    private RevTouchSensor armResetTouchSensor;


    ////Variables////
    //Calibration
    private double armExtension = 0.5; //in meters

    public SchrodingerControl(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, useChassis, usePayload, useNavigator);
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===
        if(USE_PAYLOAD) {
            Servo gripperR = currentOpMode.hardwareMap.servo.get("gr");
            Servo gripperL = currentOpMode.hardwareMap.servo.get("gl");
            Servo headingServo = currentOpMode.hardwareMap.servo.get("gh");
            Servo rotationServoR = currentOpMode.hardwareMap.servo.get("rr");
            Servo rotationServoL = currentOpMode.hardwareMap.servo.get("rl");

            Servo armServoR = currentOpMode.hardwareMap.servo.get("ar");
            Servo armServoL = currentOpMode.hardwareMap.servo.get("al");
            DcMotor armMotor = currentOpMode.hardwareMap.dcMotor.get("AM");

            Servo grabberServoR = currentOpMode.hardwareMap.servo.get("fr");
            Servo grabberServoL = currentOpMode.hardwareMap.servo.get("fl");
            DcMotor TM = currentOpMode.hardwareMap.dcMotor.get("TM");

            DcMotor intakeMotorR = currentOpMode.hardwareMap.dcMotor.get("IR");
            DcMotor intakeMotorL = currentOpMode.hardwareMap.dcMotor.get("IL");

            arm = new SchrodingerArm(armMotor, armServoR, armServoL);
            gripper = new SchrodingerGripper(gripperR, gripperL, headingServo, rotationServoR, rotationServoL);
            grabbers = new SchrodingerFoundationGrabbers(grabberServoR, grabberServoL,TM);

            intake = new MotorizedIntake();
            intake.Init(new DcMotor[]{intakeMotorR, intakeMotorL}, new double[]{1,-1});

            armResetTouchSensor = currentOpMode.hardwareMap.get(RevTouchSensor.class, "armReset");
        }

        //TODO ===INIT CORE ROBOT===
        super.InitCoreRobotModules();

        if(USE_CHASSIS) {
            odometry = new DemoBotOdometry(FR, FL);
        }
    }

    public void Start(){
        super.StartCoreRobotModules();
    }

    //CALLABLE METHODS//
    public void ChangeArmRotation(double speed){arm.ChangeRotation(speed);}
    public void ChangeArmExtension(double speed){arm.ChangeExtension(speed);}
    public void ArmToIntake(){
        gripper.SetGripperState(true);
        gripper.SetTargetRotation(180);
        arm.ArmToZero();
    }
    public void ArmToPlace(int stackHeight){
        gripper.SetTargetRotation(0);
        gripper.SetGripperState(true);
        arm.SetTargetRotation(-130);
    }
    public void ChangeGripperRotation(double speed){gripper.ChangeTargetRotation(speed);}
    public void SetGripperState(boolean closed){gripper.SetGripperState(closed);}
    public void SwitchGripperState(){gripper.SwitchGripperState();}
    public void SetFoundationGrabberState(boolean down){grabbers.SetGrabberState(down);}
    public void SwitchFoundationGrabberState(){grabbers.SwitchGrabberState();}
    public void Intake(double power) {
        //Runs the intake of the robot
        intake.SetIntakePower(power);
    }
    public void CheckIfArmNeedsReset(){
        if(armResetTouchSensor.isPressed() && arm.armMotor.getCurrentPosition() != 0)arm.ResetArmMotor();

    }
    public void PrintTelemetry(){
        gripper.PrintTelemetry(currentOpMode.telemetry);
        grabbers.PrintTelemetry(currentOpMode.telemetry);
        arm.PrintTelemetry(currentOpMode.telemetry);
    }

    //PUBLIC GETTERS
    public SchrodingerGripper GetGripper(){return gripper;}
    public SchrodingerArm GetArm(){return arm;}
    public MotorizedIntake GetIntake(){return intake;}
    public SchrodingerFoundationGrabbers GetFoundationGrabbers(){return grabbers;}
    public DemoBotOdometry GetOdometry() {return odometry;}
}
