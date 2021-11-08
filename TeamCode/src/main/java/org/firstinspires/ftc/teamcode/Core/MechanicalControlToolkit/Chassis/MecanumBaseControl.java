package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.PIDController;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Orion.NavProfiles.NavigationProfile;
import org.firstinspires.ftc.teamcode.Orion.OrionNavigator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;

@Config
public class MecanumBaseControl
{
    ////Calibration////


    ////Dependencies////
    //Mechanical Components
    protected MecanumChassis chassis;
    //Core
    protected PIDController pidController; //Look here: https://github.com/tekdemo/MiniPID-Java for how to use it
    protected IMU imu;
    public HermesLog log;
    //Orion Navigator
    protected OrionNavigator orion;
    protected NavigationProfile navigationProfile;
    //OpMode
    protected OpMode opMode;

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
    public MecanumBaseControl(OpMode setOpMode, NavigationProfile setNavProfile, HermesLog setLog, boolean useChassis, boolean usePayload, boolean useNavigator)
    {
        opMode = setOpMode;
        USE_CHASSIS = useChassis;
        USE_PAYLOAD = usePayload;
        USE_NAVIGATOR = useNavigator;
        navigationProfile = setNavProfile;
        log = setLog;

        //TODO: ==INIT CORE MODULES==
        imu = new IMU(opMode);
        pidController = new PIDController(0,0,0);

        if(USE_NAVIGATOR) {
            //TODO: ===INIT ORION===
            orion = new OrionNavigator(opMode, this, navigationProfile);
            orion.Init();
        }

        //TODO: ===INIT CHASSIS===
        if(USE_CHASSIS) {
            DcMotor FR = opMode.hardwareMap.dcMotor.get("FR");
            DcMotor FL = opMode.hardwareMap.dcMotor.get("FL");
            DcMotor RR = opMode.hardwareMap.dcMotor.get("RR");
            DcMotor RL = opMode.hardwareMap.dcMotor.get("RL");
            chassis = new MecanumChassis(imu, FR, FL, RR, RL, opMode.telemetry, false);//Create chassis instance w/ motors
        }
    }

    //TODO: Call this on Init()
    public void InitCoreRobotModules(){

    }

    //TODO: Call this on Start()
    public void StartCoreRobotModules(){
        imu.Start();
        imu.ResetGyro();
    }

    //TODO: Call this on Loop()
    public void Update(){

        if(isUSE_NAVIGATOR()) {
            orion.Update();
            log.AddData(new Object[]{new RobotPose(orion.GetPose().getX(), orion.GetPose().getY(), orion.GetPose().getHeading())});
        }
        log.Update();
    }

    //TODO: UNIVERSAL PUBLIC METHODS
    public void RawDrive(double inputAngle, double speed, double turnOffset){
        double finalAngle = inputAngle;
        if(headlessMode) finalAngle += imu.GetRobotAngle();
        opMode.telemetry.addData("ROBOT ANGLE ", imu.GetRobotAngle());
        opMode.telemetry.addData("FINAL ANGLE ", finalAngle);

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
        chassis.EncoderBrake();
    }

    //TODO: UNIVERSAL GETTERS
    public OrionNavigator GetOrion(){return orion;}
    public IMU GetImu(){return imu;}
    public MecanumChassis GetChassis(){return chassis;}
    public PIDController GetPID(){return chassis.GetHeadingPID();}
    public OpMode GetOpMode(){return opMode;}

    //TODO: SETTER METHODS
    public void SetHeadingPID(double p, double i, double d){
        chassis.SetHeadingPID(p,i,d);
    }

    //TODO: PRIVATE METHODS

}
