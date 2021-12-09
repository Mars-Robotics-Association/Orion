package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis;

//REQUIRED TO COMPILE: Phones | REV Hub
//REQUIRED TO RUN: Chassis

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;

public class TankChassisControl
{
    ////Dependencies////
    protected TankChassis chassis;
    protected IMU imu;
    public HermesLog log;
    protected OpMode currentOpMode;

    ////Util////
    protected boolean headlessMode = false;

    ////Robot Config////
    protected boolean USE_CHASSIS = true;
    protected boolean USE_PAYLOAD = false;
    protected boolean USE_NAVIGATOR = false;
    public boolean isUSE_CHASSIS(){return USE_CHASSIS;}
    public boolean isUSE_PAYLOAD(){return USE_PAYLOAD;}
    public boolean isUSE_NAVIGATOR(){return USE_NAVIGATOR;}

    //Initializer
    public TankChassisControl(OpMode setOpMode, HermesLog setLog, boolean useChassis, boolean usePayload, boolean useNavigator)
    {
        //initialize things
        currentOpMode = setOpMode;
        USE_CHASSIS = useChassis;
        USE_PAYLOAD = usePayload;
        USE_NAVIGATOR = useNavigator;
        //navigationProfile = setNavProfile;
        log = setLog;

        imu = new IMU(currentOpMode);

        if(USE_NAVIGATOR){
            //TODO: add navigator capabilities
        }

        if(USE_CHASSIS){
            DcMotor r = currentOpMode.hardwareMap.dcMotor.get("R");
            DcMotor l = currentOpMode.hardwareMap.dcMotor.get("L");
            l.setDirection(DcMotorSimple.Direction.REVERSE);
            chassis = new TankChassis(imu, r, l, currentOpMode.telemetry, false);
        }
    }

    //Call on Start()
    public void StartCoreRobotModules(){
        imu.Start();
        imu.ResetGyro();
    }

    //Call on Loop()
    public void Update(){}

    ////Drive Functions////
    public void Drive(double speedNormal, double speedHeadless, double targetHeading, double spotTurnFactor, double sweepTurnFactor){
        if(headlessMode) DriveHeadless(speedHeadless, targetHeading, spotTurnFactor, sweepTurnFactor);
        else DriveNormal(speedNormal, sweepTurnFactor);
    }

    public void DriveNormal(double speed, double turnFactor){chassis.DriveNormal(speed, turnFactor);}

    public void DriveHeadless(double speed, double targetHeading, double spotTurnFactor, double sweepTurnFactor){chassis.DriveHeadless(speed, targetHeading, spotTurnFactor, sweepTurnFactor);}

    public void ResetGyro(){
        //Offsets the gryo so the current heading can be zero with GetRobotAngle()
        imu.ResetGyro();
    }

    //toggles headless mode on or off
    public void SwitchHeadlessMode(){headlessMode = !headlessMode;}

    ////Getters/////
    public IMU GetImu(){return imu;}
    public TankChassis GetChassis(){return chassis;}

    ////Setters////
    public void SetHeadingPID(double p, double i, double d, boolean reverse){
        chassis.SetHeadingPID(p,i,d, reverse);
    }
}
