package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.MotorArray;
import org.firstinspires.ftc.teamcode.Core.PIDController;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.RobotPose;
import org.firstinspires.ftc.teamcode.Orion.NavProfiles.NavigationProfile;
import org.firstinspires.ftc.teamcode.Orion.OrionNavigator;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;

@Config
public class MecanumChassis
{
    ////Dependencies////
    protected IMU imu;
    public HermesLog log;
    private PIDController headingPIDController;
    private PIDController speedPID;
    private PIDController directionPID;
    protected OpMode opMode;
    //Motors
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor RR;
    private DcMotor RL;
    private MotorArray driveMotors;

    //Brake pos
    private int FRBrakePos = 0;
    private int FLBrakePos = 0;
    private int RRBrakePos = 0;
    private int RLBrakePos = 0;

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
    public MecanumChassis(OpMode setOpMode, NavigationProfile setNavProfile, HermesLog setLog, boolean useChassis, boolean usePayload, boolean useNavigator, boolean useEncoders)
    {
        opMode = setOpMode;
        USE_CHASSIS = useChassis;
        USE_PAYLOAD = usePayload;
        USE_NAVIGATOR = useNavigator;
        log = setLog;

        //TODO: ==INIT CORE MODULES==
        imu = new IMU(opMode);

        if(USE_NAVIGATOR) {
            //TODO: ===INIT ORION===
        }

        //TODO: ===INIT CHASSIS===
        if(USE_CHASSIS) {
            FR = opMode.hardwareMap.dcMotor.get("FR");
            FL = opMode.hardwareMap.dcMotor.get("FL");
            RR = opMode.hardwareMap.dcMotor.get("RR");
            RL = opMode.hardwareMap.dcMotor.get("RL");
            driveMotors = new MotorArray(new DcMotor[]{FR,FL,RR,FL}, new double[]{1,1,1,1}, useEncoders);

            driveMotors.StopAndResetEncoders();
            if(useEncoders) driveMotors.RunWithEncodersMode();
            else driveMotors.RunWithoutEncodersMode();
            driveMotors.SetMotorPowers(new double[]{0,0,0,0});

            headingPIDController = new PIDController(0,0,0);//Create the pid controller.
            speedPID = new PIDController(0,0,0);//Create the pid controller.
            directionPID = new PIDController(0,0,0);//Create the pid controller.
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
        }
        log.Update();
    }

    //TODO: UNIVERSAL PUBLIC METHODS
    public void DriveWithGamepad(ControllerInput gamepad, double driveSpeed, double turnSpeed, double speedMultiplier) {
        //MOVE if left joystick magnitude > 0.1
        if (gamepad.CalculateLJSMag() > 0.1) {
            RawDrive(gamepad.CalculateLJSAngle(), gamepad.CalculateLJSMag() * driveSpeed * speedMultiplier, gamepad.GetRJSX() * turnSpeed * speedMultiplier);//drives at (angle, speed, turnOffset)
            opMode.telemetry.addData("Moving at ", gamepad.CalculateLJSAngle());
        }
        //TURN if right joystick magnitude > 0.1 and not moving
        else if (Math.abs(gamepad.GetRJSX()) > 0.1) {
            RawTurn(gamepad.GetRJSX() * turnSpeed * speedMultiplier);//turns at speed according to rjs1
            opMode.telemetry.addData("Turning", true);
        }
        else {
            SetMotorSpeeds(0,0,0,0);
        }
    }

    //Tells robot to raw move at any angle. Turn speed variable causes it to sweep turn / corkscrew.
    //This is called continuously while the robot is driving.
    public void RawDrive(double inputAngle, double speed, double turnOffset){
        double finalAngle = inputAngle;
        if(headlessMode) finalAngle += imu.GetRobotAngle();
        opMode.telemetry.addData("ROBOT ANGLE ", imu.GetRobotAngle());
        opMode.telemetry.addData("FINAL ANGLE ", finalAngle);

        //Sets the mode so that robot can drive and record encoder values
        driveMotors.RunWithoutEncodersMode();

        //HEADING PID//
        //Uses pid controller to correct for heading error using (currentAngle, targetAngle)
        double headingPIDOffset = headingPIDController.getOutput(turnOffset, imu.GetAngularVelocity());
        //if the number is not real, reset pid controller
        if(!(headingPIDOffset > 0 || headingPIDOffset <= 0)){
            headingPIDController.reset();
        }

        //TRANSLATIONAL PID//
        double velocityMag = 0; //TODO: fill out
        double velocityDir = 0; //TODO: fill out
        double magPIDOffset = speedPID.getOutput(speed, velocityMag);
        double dirPIDOffset = directionPID.getOutput(finalAngle, velocityDir);


        //set the powers of the motors with pid offset applied
        //TODO remove line below
        headingPIDOffset *= -1;

        //Gets speeds for the motors
        double[] speeds = CalculateWheelSpeedsTurning(finalAngle, speed, turnOffset+headingPIDOffset);
        //SetMotorSpeeds(speeds[0]+headingPIDOffset, speeds[1]+headingPIDOffset, speeds[2]+headingPIDOffset, speeds[3]+headingPIDOffset);
        SetMotorSpeeds(speeds[0], -speeds[1], speeds[2], -speeds[3]);

        //Updates brake pos, as this is called continuously as robot is driving
        UpdateEncoderBrakePos();
    }
    public void RawTurn(double speed){
        //Used continuously in teleop to turn the robot
        //Enter speed for turn- positive speed turns left, negative right

        //Use motors and record encoder values
        driveMotors.RunWithoutEncodersMode();

        //Set motor speeds all equal, as this causes it to do a spot turn
        SetMotorSpeeds(speed, -speed, speed, -speed);

        //Update the values for breaking
        UpdateEncoderBrakePos();
    }
    public void ResetGyro(){
        //Offsets the gryo so the current heading can be zero with GetRobotAngle()
        //gyroOffset = imu.GetRawAngles().firstAngle;
        imu.ResetGyro();
    }
    public void SwitchHeadlessMode(){headlessMode = !headlessMode;}

    //TODO: UNIVERSAL GETTERS
    public IMU GetImu(){return imu;}
    public OpMode GetOpMode(){return opMode;}
    public PIDController GetHeadingPID() {
        return headingPIDController;
    }
    public PIDController GetVelocityPID() {
        return speedPID;
    }

    //TODO: PRIVATE METHODS
    //Utility
    public void SetMotorSpeeds(double fr, double fl, double rr, double rl){
        driveMotors.SetMotorPowers(new double[]{fr, -fl, rr, -rl});
    }
    public void UpdateEncoderBrakePos(){
        //Update the values for breaking
        FRBrakePos = FR.getCurrentPosition();
        FLBrakePos = FL.getCurrentPosition();
        RRBrakePos = RR.getCurrentPosition();
        RLBrakePos = RL.getCurrentPosition();
    }
    public void EncoderBrake(){
        //Stop the robot and hold position. Meant to be called once
        UpdateEncoderBrakePos();
        driveMotors.SetTargetPositions(new int[] {FRBrakePos, FLBrakePos, RRBrakePos, RLBrakePos},true);
        SetMotorSpeeds(0.5,-0.5,0.5,-0.5);
    }

    public void SetHeadingPID(double p, double i, double d){
        headingPIDController.setPID(p,i,d);
    }
    public void SetSpeedPID(double p, double i, double d){
        speedPID.setPID(p,i,d);
    }
    public void SetDirectionPID(double p, double i, double d){
        directionPID.setPID(p,i,d);
    }

    public double[] CalculateWheelSpeedsTurning(double degrees, double speed, double turnSpeed)
    {
        //Returns the speeds the motors need to move at to move. A negative turn speed turns right, a positive left.

        /*Wheel speed is calculated by getting the cosine wave (which we found matches the speed that
         * the wheels need to go) with a positive 45 or negative 45 shift, depending on the wheel. This works
         * so that no matter the degrees, it will always come out with the right value. A turn offset is added
         * to the end for corkscrewing, or turning while driving*/
        double FRP = -Math.cos(Math.toRadians(degrees + 45)) * speed + turnSpeed;
        double FLP = Math.cos(Math.toRadians(degrees - 45)) * speed + turnSpeed;
        double RRP = -Math.cos(Math.toRadians(degrees - 45)) * speed + turnSpeed;
        double RLP = Math.cos(Math.toRadians(degrees + 45)) * speed + turnSpeed;

        double[] vals = {FRP, FLP, RRP, RLP};
        return vals;
    }


}
