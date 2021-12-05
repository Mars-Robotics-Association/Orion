package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments.EncoderActuator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;

/**
 * Control class for the Belinda Robot. Controls payload.
 * Required to run: Phones | REV Hub | Belinda Chassis
 * Suggested to run: Shooter | Intake | Odometry | Webcam
 */
//The class used to control the demobot. Autonomous functions, opmodes, and other scripts can call
//methods in here to control the demobot.

//REQUIRED TO RUN: Phones | REV Hub | Demobot Chassis | Shooter | Odometry Unit

@Config
public class CuriosityRobot extends MecanumChassis
{
    ////Dependencies////
    //Mechanical Components
    CuriosityTurretArm turretArm;
    DuckSpinner duckSpinner;

    ////Variables////
    //Calibration
    private double levelPitchThreshold = 5;


    /**@param setOpMode pass the opmode running this down to access hardware map
     * @param useChassis whether to use the chassis of the robot
     * @param usePayload whether to use the shooter/intake/lift of the robot
     * @param useNavigator whether to use Orion (webcams + odometry navigation)
     */
    public CuriosityRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new CuriosityChassisProfile(), new HermesLog("Curiosity", 500, setOpMode), useChassis, usePayload, useNavigator);

        if(usePayload){
            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("Arm");
            DcMotor turretMotor = opMode.hardwareMap.dcMotor.get("Turret");
            Servo duckServo = opMode.hardwareMap.servo.get("duck");
            Servo spinnerServo = opMode.hardwareMap.servo.get("intake");

            turretArm = new CuriosityTurretArm(opMode, new ArmProfile(armMotor), new TurretProfile(turretMotor), spinnerServo,false);
            turretArm.Arm().ResetToZero();

            duckSpinner = new DuckSpinner(duckServo, 1);
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

    public CuriosityTurretArm TurretArm(){return turretArm;}
    public EncoderActuator Turret(){return turretArm.Turret();}
    public EncoderActuator Arm(){return turretArm.Arm();}

    public DuckSpinner DuckSpinner(){return duckSpinner;}

    public boolean IsRobotLevel(){
        double pitch = imu.GetRawAngles().secondAngle;
        opMode.telemetry.addData("Robot pitch: ", pitch);

        if(Math.abs(pitch)>5) return false;
        else return true;
    }
}
