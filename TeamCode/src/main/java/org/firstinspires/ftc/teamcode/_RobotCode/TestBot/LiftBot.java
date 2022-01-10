package org.firstinspires.ftc.teamcode._RobotCode.TestBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavigation;
import org.firstinspires.ftc.teamcode._RobotCode._Defaults.DefaultNavProfile;

/**
 * Control class for the Belinda Robot. Controls payload.
 * Required to run: Phones | REV Hub | Belinda Chassis
 * Suggested to run: Shooter | Intake | Odometry | Webcam
 */
//The class used to control the demobot. Autonomous functions, opmodes, and other scripts can call
//methods in here to control the demobot.

//REQUIRED TO RUN:

@Config
public class LiftBot extends MecanumChassis
{
    ////Dependencies////
    //Mechanical Components
    private DcMotor lift;
    private DcMotor intake;

    //Nav Modules
    FreightFrenzyNavigation navigation;

    ////Variables////
    public static double encoderResolution = 537.7; //gobilda 19.2:1
    public static double encoderMultiplier = 1;
    public static boolean useEncoder = false;

    //Calibration

    /**@param setOpMode pass the opmode running this down to access hardware map
     * @param useChassis whether to use the chassis of the robot
     * @param usePayload whether to use the shooter/intake/lift of the robot
     * @param useNavigator whether to use Orion (webcams + odometry navigation)
     */
    public LiftBot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new _LiftBotChassisProfile(), new HermesLog("LiftBot", 500, setOpMode), useChassis, usePayload, useNavigator);

        if(isUSE_PAYLOAD()){
            lift = opMode.hardwareMap.dcMotor.get("liftMotor");
            //liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intake = opMode.hardwareMap.dcMotor.get("intakeMotor");
        }

        if(isUSE_NAVIGATOR()){
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

    public void Update(){
        if(USE_NAVIGATOR){
        }
    }

    public void setLiftPower(double power){
        if(isUSE_PAYLOAD()) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(power);
        }
    }
    public void setIntakePower(double power){
        intake.setPower(power);
    }

    public double getLiftPosition(){
        return lift.getCurrentPosition()*encoderMultiplier/encoderResolution;
    }
}
