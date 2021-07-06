package org.firstinspires.ftc.teamcode.Core.BaseRobots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.PIDController;
import org.firstinspires.ftc.teamcode.MechanicalControl.MecanumChassis;
import org.firstinspires.ftc.teamcode.Orion.OrionNavigator;
import org.firstinspires.ftc.teamcode.Sensors.IMU;

@Config
public class MecanumBaseControl
{
    ////Calibration////
    //Movement Towards Discs
    public static double discMoveCoefficient = -0.0015;
    public static double discTurnCoefficient = -0.0015;
    public static double discMoveSpeed = 0.2;
    //Turn to Vumark
    public static double vumarkTurnCoefficient = -0.05;
    public static int targetVumarkID = 0;


    ////Dependencies////
    //Mechanical Components
    protected MecanumChassis chassis;
    //Core
    protected PIDController pidController; //Look here: https://github.com/tekdemo/MiniPID-Java for how to use it
    protected IMU imu;
    //Orion Navigator
    protected OrionNavigator orion;
    //OpMode
    protected OpMode currentOpMode;
    //Drive Motors
    protected DcMotor FR;
    protected DcMotor FL;
    protected DcMotor RR;
    protected DcMotor RL;

    //Util
    protected double gyroOffset;
    protected boolean headlessMode = false;

    //TODO: ===ROBOT CONFIGURATION===
    protected boolean USE_CHASSIS = true;
    protected boolean USE_PAYLOAD = false;
    protected boolean USE_NAVIGATOR = false;
    public boolean isUSE_CHASSIS(){return USE_CHASSIS;}
    public boolean isUSE_PAYLOAD(){return USE_PAYLOAD;}
    public boolean isUSE_NAVIGATOR(){return USE_NAVIGATOR;}


    //Initializer
    public MecanumBaseControl(OpMode setOpMode, boolean useChassis, boolean usePayload, boolean useNavigator)
    {
        currentOpMode = setOpMode;
        USE_CHASSIS = useChassis;
        USE_PAYLOAD = usePayload;
        USE_NAVIGATOR = useNavigator;
    }

    //TODO: Call this on Init()
    public void InitCoreRobotModules(){
        //TODO: ==INIT CORE MODULES==
        imu = new IMU(currentOpMode);
        pidController = new PIDController(0,0,0);

        if(USE_NAVIGATOR) {
            //TODO: ===INIT ORION===
            orion = new OrionNavigator(currentOpMode, this);
            orion.Init();
        }

        //TODO: ===INIT CHASSIS===
        if(USE_CHASSIS) {
            FR = currentOpMode.hardwareMap.dcMotor.get("FR");
            FL = currentOpMode.hardwareMap.dcMotor.get("FL");
            RR = currentOpMode.hardwareMap.dcMotor.get("RR");
            RL = currentOpMode.hardwareMap.dcMotor.get("RL");
        }
        if(USE_CHASSIS) {
            chassis = new MecanumChassis(imu, FR, FL, RR, RL, currentOpMode.telemetry, false, true);//Create chassis instance w/ motors
            chassis.Init();
        }
    }

    //TODO: Call this on Start()
    public void StartCoreRobotModules(){
        imu.Start();
        imu.ResetGyro();
    }

    //TODO: Call this on Loop()
    public void Update(){

        if(isUSE_NAVIGATOR()) orion.Update();
    }

    //TODO: UNIVERSAL PUBLIC METHODS
    public void RawDrive(double inputAngle, double speed, double turnOffset){
        double finalAngle = inputAngle;
        if(headlessMode) finalAngle += imu.GetRobotAngle();
        currentOpMode.telemetry.addData("ROBOT ANGLE ", imu.GetRobotAngle());
        currentOpMode.telemetry.addData("FINAL ANGLE ", finalAngle);

        chassis.MoveAtAngle(finalAngle, speed, turnOffset);
    }
    public void RawTurn(double speed){
        //Used continuously in teleop to turn the robot
        //Enter speed for turn- positive speed turns left, negative right
        chassis.SpotTurn(speed);
    }
    public void ResetGyro(){
        //Offsets the gryo so the current heading can be zero with GetRobotAngle()
        //gyroOffset = imu.GetRawAngles().firstAngle;
        imu.ResetGyro();
    }
    public void SwitchHeadlessMode(){headlessMode = !headlessMode;}
    public void Brake(){
        //Called once to brake the robot
        chassis.Brake();
    }

    public void MoveTowardsClosestDisc(){ orion.MoveTowardsDiscRaw(discMoveSpeed, discMoveCoefficient); }
    public double TurnTowardsClosestDiscSpeed(){return orion.TurnTowardsDiscSpeed(discTurnCoefficient);}

    //public void TurnTowardsVuMark(){orion.TurnTowardsVuMark(1, targetVumarkID, vumarkTurnCoefficient, true);}
    public void SetOriginToVumark(int vumarkIndex){ orion.SetOriginToVumark(vumarkIndex);}

    //TODO: UNIVERSAL GETTERS
    public OrionNavigator GetOrion(){return orion;}
    public IMU GetImu(){return imu;}
    public MecanumChassis GetChassis(){return chassis;}
    public PIDController GetPID(){return chassis.GetHeadingPID();}
    public OpMode GetOpMode(){return currentOpMode;}

    //TODO: SETTER METHODS
    public void SetDrivePID(double p, double i, double d){
        chassis.SetHeadingPID(p,i,d);
    }

    //TODO: PRIVATE METHODS

}
