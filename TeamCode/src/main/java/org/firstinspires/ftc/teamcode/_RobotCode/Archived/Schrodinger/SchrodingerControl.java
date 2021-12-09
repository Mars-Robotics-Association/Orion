package org.firstinspires.ftc.teamcode._RobotCode.Archived.Schrodinger;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.Archived.MecanumBaseControl;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.drive.opmode.DefaultNavProfile;
import org.firstinspires.ftc.teamcode._RobotCode.Archived.Schrodinger.MechanicalControllers.SchrodingerArm;
import org.firstinspires.ftc.teamcode._RobotCode.Archived.Schrodinger.MechanicalControllers.SchrodingerFoundationGrabbers;
import org.firstinspires.ftc.teamcode._RobotCode.Archived.Schrodinger.MechanicalControllers.SchrodingerGripper;

//The class used to control schrodinger. Autonomous functions, opmodes, and other scripts can call
//methods in here to control the schrodinger.

//REQUIRED TO RUN: Phones | REV Hub | Demobot Chassis | Shooter | Odometry Unit

public class SchrodingerControl extends MecanumBaseControl
{
    ////Dependencies////
    //Mechanical Components
    private SchrodingerArm arm;
    private SchrodingerGripper gripper;
    private SchrodingerFoundationGrabbers grabbers;

    //Sensors
    private RevTouchSensor armResetTouchSensor;


    ////Variables////
    //Calibration
    private double armExtension = 0.5; //in meters

    public SchrodingerControl(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new DefaultNavProfile(), new HermesLog("ShrodingerControl", 500, setOpMode), useChassis, usePayload, useNavigator);
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===
        if(USE_PAYLOAD) {
            Servo gripperR = opMode.hardwareMap.servo.get("gr");
            Servo gripperL = opMode.hardwareMap.servo.get("gl");
            Servo headingServo = opMode.hardwareMap.servo.get("gh");
            Servo rotationServoR = opMode.hardwareMap.servo.get("rr");
            Servo rotationServoL = opMode.hardwareMap.servo.get("rl");

            Servo armServoR = opMode.hardwareMap.servo.get("ar");
            Servo armServoL = opMode.hardwareMap.servo.get("al");
            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("AM");

            Servo grabberServoR = opMode.hardwareMap.servo.get("fr");
            Servo grabberServoL = opMode.hardwareMap.servo.get("fl");
            DcMotor TM = opMode.hardwareMap.dcMotor.get("TM");

            DcMotor intakeMotorR = opMode.hardwareMap.dcMotor.get("IR");
            DcMotor intakeMotorL = opMode.hardwareMap.dcMotor.get("IL");

            arm = new SchrodingerArm(armMotor, armServoR, armServoL);
            gripper = new SchrodingerGripper(gripperR, gripperL, headingServo, rotationServoR, rotationServoL);
            grabbers = new SchrodingerFoundationGrabbers(grabberServoR, grabberServoL,TM);


            armResetTouchSensor = opMode.hardwareMap.get(RevTouchSensor.class, "armReset");
        }

        //TODO ===INIT CORE ROBOT===
        super.InitCoreRobotModules();

        if(USE_CHASSIS) {
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
    }
    public void CheckIfArmNeedsReset(){
        if(armResetTouchSensor.isPressed() && arm.armMotor.getCurrentPosition() != 0)arm.ResetArmMotor();

    }
    public void PrintTelemetry(){
        gripper.PrintTelemetry(opMode.telemetry);
        grabbers.PrintTelemetry(opMode.telemetry);
        arm.PrintTelemetry(opMode.telemetry);
    }

    //PUBLIC GETTERS
    public SchrodingerGripper GetGripper(){return gripper;}
    public SchrodingerArm GetArm(){return arm;}
    public SchrodingerFoundationGrabbers GetFoundationGrabbers(){return grabbers;}
}
