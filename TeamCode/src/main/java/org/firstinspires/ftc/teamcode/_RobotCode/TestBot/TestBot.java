package org.firstinspires.ftc.teamcode._RobotCode.TestBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumBaseControl;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode._RobotCode._Defaults.DefaultNavProfile;

/**
 * Control class for the Belinda Robot. Controls payload.
 * Required to run: Phones | REV Hub | Belinda Chassis
 * Suggested to run: Shooter | Intake | Odometry | Webcam
 */
//The class used to control the demobot. Autonomous functions, opmodes, and other scripts can call
//methods in here to control the demobot.

//REQUIRED TO RUN: Phones | REV Hub | Demobot Chassis | Shooter | Odometry Unit

@Config
public class TestBot extends MecanumBaseControl
{
    ////Dependencies////
    //Mechanical Components
    ArmControl arm;
    TestBotNavCore navCore;

    ////Variables////
    //Calibration
    private double levelPitchThreshold = 5;


    /**@param setOpMode pass the opmode running this down to access hardware map
     * @param useChassis whether to use the chassis of the robot
     * @param usePayload whether to use the shooter/intake/lift of the robot
     * @param useNavigator whether to use Orion (webcams + odometry navigation)
     */
    public TestBot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new DefaultNavProfile(), new HermesLog("Curiosity", 500, setOpMode), useChassis, usePayload, useNavigator);
        navCore = new TestBotNavCore(); //feed in nav modules here

        if(usePayload){
            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("Arm");
            Servo spinnerServo = opMode.hardwareMap.servo.get("spinner");
            arm = new ArmControl(opMode,armMotor,spinnerServo,false,true);
            arm.ResetArmPos();
        }
    }

    //SETUP METHODS//
    public void Init(){
        //TODO ===INIT PAYLOAD===

        //TODO ===INIT CORE ROBOT===
        super.InitCoreRobotModules();


        if(USE_NAVIGATOR) {
        }
    }

    public void Start(){
        super.StartCoreRobotModules();
    }

    public ArmControl Arm(){return arm;}

    public boolean IsRobotLevel(){
        double pitch = imu.GetRawAngles().secondAngle;
        opMode.telemetry.addData("Robot pitch: ", pitch);

        if(Math.abs(pitch)>5) return false;
        else return true;
    }
}
