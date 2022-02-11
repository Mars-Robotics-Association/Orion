package org.firstinspires.ftc.teamcode._RobotCode.Curiosity;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.BlinkinController;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Orion.NavModules.FreightFrenzy.FreightFrenzyNavigation;

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
    DistanceSensor duckDist;
    EncoderActuator tapeMeasureCapper;
    BlinkinController blinkinController;

    //Misc
    TextCycler text;
    FtcDashboard dashboard;


    //Nav Modules
    FreightFrenzyNavigation navigation;

    public boolean isBlue = true;

    ////Variables////
    //Calibration
    private double levelPitchThreshold = 5;


    /**@param setOpMode pass the opmode running this down to access hardware map
     * @param useChassis whether to use the chassis of the robot
     * @param usePayload whether to use the shooter/intake/lift of the robot
     * @param useNavigator whether to use Orion (webcams + odometry navigation)
     */
    public CuriosityRobot(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator) {
        super(setOpMode, new _ChassisProfile(), new HermesLog("Curiosity", 500, setOpMode), useChassis, usePayload, useNavigator);

        duckDist = opMode.hardwareMap.get(DistanceSensor.class, "duckDist");
        DistanceSensor intakeDist = opMode.hardwareMap.get(DistanceSensor.class, "intakeDist");
        DistanceSensor armResetDist = opMode.hardwareMap.get(DistanceSensor.class, "armResetDist");

        text = new TextCycler(opMode);
        dashboard = FtcDashboard.getInstance();


        if(USE_PAYLOAD){
            DcMotor armMotor = opMode.hardwareMap.dcMotor.get("Arm");
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            //DcMotor tapeMotor = opMode.hardwareMap.dcMotor.get("Tape");
            DcMotor turretMotor = opMode.hardwareMap.dcMotor.get("Turret");
            DcMotor duckMotor = opMode.hardwareMap.dcMotor.get("Duck");
            Servo spinnerServo = opMode.hardwareMap.servo.get("intake");

            TouchSensor armTouch = opMode.hardwareMap.get(TouchSensor.class, "armTouch");

            blinkinController = new BlinkinController(opMode);

            turretArm = new CuriosityTurretArm(opMode, blinkinController, new _ArmProfile(armMotor), new _TurretProfile(turretMotor), spinnerServo, intakeDist, armResetDist,armTouch,false);
            turretArm.Arm().ResetToZero();

            duckSpinner = new DuckSpinner(duckMotor, 1);

            //tapeMeasureCapper = new EncoderActuator(opMode,new _TapeMeasureProfile(tapeMotor));
        }

        if(useNavigator){
            DistanceSensor portDist = opMode.hardwareMap.get(DistanceSensor.class, "portDist");
            DistanceSensor starboardDist = opMode.hardwareMap.get(DistanceSensor.class, "starboardDist");
            ColorSensor colorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
            navigation = new FreightFrenzyNavigation(opMode, this, turretArm, duckSpinner, duckDist, intakeDist, armResetDist, portDist, starboardDist, colorSensor, blinkinController, FreightFrenzyNavigation.AllianceSide.BLUE);
            navigation.SetThread(new Thread(navigation));
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
        navigation.NavigatorOn();
    }

    public void Update(){
        text.Update();
        if(USE_PAYLOAD){
            turretArm.UpdateIntakeTiered();
        }
    }

    public void Stop(){
        navigation.StopNavigator();
    }

    public CuriosityTurretArm TurretArm(){return turretArm;}
    //public EncoderActuator Turret(){return turretArm.Turret();}
    public EncoderActuator Arm(){return turretArm.Arm();}

    public DuckSpinner GetDuckSpinner(){return duckSpinner;}

    public FreightFrenzyNavigation Navigation(){return navigation;}

    public BlinkinController Lights(){return blinkinController;}

    public double GetDistToWallCM(){return duckDist.getDistance(DistanceUnit.CM);}

    public boolean IsRobotLevel(){
        double pitch = imu.GetRawAngles().secondAngle;
        opMode.telemetry.addData("Robot pitch: ", pitch);

        if(Math.abs(pitch)>5) return false;
        else return true;
    }
}
