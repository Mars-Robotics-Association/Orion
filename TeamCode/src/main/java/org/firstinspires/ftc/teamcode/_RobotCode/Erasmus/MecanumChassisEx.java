package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.IMU;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.PIDController;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;

@Config
public class MecanumChassisEx
{
    ////Dependencies////
    protected IMU imu;
    private BaseRobot baseRobot;
    public HermesLog log;
    private PIDController poseXYPID;
    private PIDController speedPID;
    private PIDController trajectoryPID;
    private PIDController poseAnglePID;
    protected OpMode opMode;
    //Motors
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor RR;
    private DcMotor RL;
    public DCMotorArray driveMotors;

    //Brake pos
    private int FRBrakePos = 0;
    private int FLBrakePos = 0;
    private int RRBrakePos = 0;
    private int RLBrakePos = 0;

    private double inputOffset = 0;

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

    public ChassisProfile profile;


    //Initializer
    public MecanumChassisEx(OpMode setOpMode, ChassisProfile setProfile, HermesLog setLog, BaseRobot setBaseRobot)
    {
        opMode = setOpMode;
        baseRobot = setBaseRobot;
        USE_CHASSIS = baseRobot.USE_CHASSIS;
        USE_PAYLOAD = baseRobot.USE_PAYLOAD;
        USE_NAVIGATOR = baseRobot.USE_NAVIGATOR;
        log = setLog;
        profile = setProfile;

        //==INIT CORE MODULES==
        imu = new IMU(opMode);

        if(USE_NAVIGATOR) {
            //===INIT ORION===
        }

        //===INIT CHASSIS===
        if(USE_CHASSIS) {
            //gets motors
            FR = opMode.hardwareMap.dcMotor.get(profile.motorNames()[0]);
            FL = opMode.hardwareMap.dcMotor.get(profile.motorNames()[1]);
            RR = opMode.hardwareMap.dcMotor.get(profile.motorNames()[2]);
            RL = opMode.hardwareMap.dcMotor.get(profile.motorNames()[3]);

            //creates motor array
            driveMotors = new DCMotorArray(new DcMotor[]{FR,FL,RR,RL}, new double[]{1,1,1,1}, profile.useEncoders());

            //reverse motors if needed
            for (int i = 0; i < 4; i++) {
                if(profile.flipMotors()[i]) driveMotors.getMotors()[i].setDirection(DcMotorSimple.Direction.REVERSE);
                else driveMotors.getMotors()[i].setDirection(DcMotorSimple.Direction.FORWARD);
            }

            //resets motors
            driveMotors.stopAndResetEncoders();
            if(profile.useEncoders()) driveMotors.runWithEncodersMode();
            else driveMotors.runWithoutEncodersMode();
            driveMotors.setPowers(new double[]{0,0,0,0});

            //setup PIDs
            poseXYPID = new PIDController(0,0,0);//Create the pid controller.
            speedPID = new PIDController(0,0,0);//Create the pid controller.
            trajectoryPID = new PIDController(0,0,0);//Create the pid controller.
            poseAnglePID = new PIDController(0,0,0);//Create the pid controller.

            //set PIDs to profile values
            setPoseXYPID(profile.poseXYPID()[0],profile.poseXYPID()[1],profile.poseXYPID()[2]);
            setSpeedPID(profile.speedPID()[0],profile.speedPID()[1],profile.speedPID()[2]);
            setTrajectoryPID(profile.trajectoryPID()[0],profile.trajectoryPID()[1],profile.trajectoryPID()[2]);
            setPoseAnglePID(profile.poseAnglePID()[0],profile.poseAnglePID()[1],profile.poseAnglePID()[2]);
        }
    }

    //Call this on Start()
    public void startChassis(){
        imu.start();
        imu.resetGyro();
    }

    //Call this on Loop()
    public void update(){
        log.Update();
    }

    //TODO: UNIVERSAL PUBLIC METHODS
    public void driveWithGamepad(ControllerInput controllerInput, double speedMultiplier) {
        //MOVE if left joystick magnitude > 0.1
        if (controllerInput.calculateLJSMag() > 0.1) {
            rawDrive(
                    controllerInput.calculateLJSAngle()+inputOffset,
                    controllerInput.calculateLJSMag() * profile.moveSpeed() * speedMultiplier,
                    controllerInput.getRJSX() * profile.turnSpeed() * speedMultiplier);//drives at (angle, speed, turnOffset)

            opMode.telemetry.addData("Moving at ", controllerInput.calculateLJSAngle());
        }
        //TURN if right joystick magnitude > 0.1 and not moving
        else if (Math.abs(controllerInput.getRJSX()) > 0.1) {
            rawTurn(controllerInput.getRJSX() * profile.turnSpeed() * speedMultiplier);//turns at speed according to rjs1
            opMode.telemetry.addData("Turning", true);
        }
        else {
            setMotorSpeeds(0,0,0,0);
        }
    }
    public void setInputOffset(double offset) {inputOffset = offset;}

    //Tells robot to raw move at any angle. Turn speed variable causes it to sweep turn / corkscrew.
    //This is called continuously while the robot is driving.
    public void rawDrive(double inputAngle, double speed, double turnOffset){
        double finalAngle = inputAngle;
        if(headlessMode) finalAngle += imu.getRobotAngle();
        opMode.telemetry.addData("ROBOT ANGLE ", imu.getRobotAngle());
        opMode.telemetry.addData("FINAL ANGLE ", finalAngle);

        //Sets the mode so that robot can drive and record encoder values
        driveMotors.runWithoutEncodersMode();

        //HEADING PID//
        //Uses pid controller to correct for heading error using (currentAngle, targetAngle)
        double headingPIDOffset = poseXYPID.getOutput(turnOffset, imu.getAngularVelocity());
        //if the number is not real, reset pid controller
        if(!(headingPIDOffset > 0 || headingPIDOffset <= 0)){
            poseXYPID.reset();
        }

        //TRANSLATIONAL PID//
        double velocityMag = 0; //TODO: fill out
        double velocityDir = 0; //TODO: fill out
        double magPIDOffset = speedPID.getOutput(speed, velocityMag);
        double dirPIDOffset = trajectoryPID.getOutput(finalAngle, velocityDir);


        //set the powers of the motors with pid offset applied
        if(profile.flipIMU()) headingPIDOffset *= -1;

        //Gets speeds for the motors
        double[] speeds = calculateWheelSpeedsTurning(finalAngle, speed, turnOffset+headingPIDOffset);
        //SetMotorSpeeds(speeds[0]+headingPIDOffset, speeds[1]+headingPIDOffset, speeds[2]+headingPIDOffset, speeds[3]+headingPIDOffset);
        setMotorSpeeds(speeds[0], -speeds[1], speeds[2], -speeds[3]);
    }
    public void rawDriveTurningTowards(double driveAngle, double speed, double facingAngle, double turnCoefficient){
        double turnOffset = getHeadingError(facingAngle)*speed*turnCoefficient;
        rawDrive(driveAngle,speed,turnOffset);
    }
    public void stop(){rawDrive(0,0,0);}
    //Used continuously in teleop to turn the robot
    //Enter speed for turn- positive speed turns left, negative right
    public void rawTurn(double speed){


        //Use motors and record encoder values
        driveMotors.runWithoutEncodersMode();

        //Set motor speeds all equal, as this causes it to do a spot turn
        setMotorSpeeds(speed, -speed, speed, -speed);
    }
    public double getHeadingError(double targetHeading){
        //fix angle
        if(targetHeading > 180) targetHeading = -360+targetHeading;
        else if(targetHeading < -180) targetHeading = 360+targetHeading;
        //calculate error and turn speed
        double error = targetHeading - imu.getRobotAngle();

        return error;
    }
    public void turnTowardsAngle(double targetHeading, double speed, double coefficient){
        double error = getHeadingError(targetHeading);
        double turnSpeed = (error*coefficient*speed) + 0.1*(error/Math.abs(error)); //add offset of 0.2
        rawTurn(turnSpeed);
    }
    public boolean inWithinRangeOfAngle(double targetHeading, double threshold){
        //fix angle
        if(targetHeading > 180) targetHeading = -360+targetHeading;
        else if(targetHeading < -180) targetHeading = 360+targetHeading;
        //calculate if within range
        double error = targetHeading - imu.getRobotAngle();
        if(Math.abs(error)<threshold) return true;
        else return false;
    }
    //Offsets the gryo so the current heading can be zero with GetRobotAngle()
    public void resetGyro(){imu.resetGyro();}
    public void switchHeadlessMode(){headlessMode = !headlessMode;}
    public void setHeadlessMode(boolean set){headlessMode = set;}

    //UNIVERSAL GETTERS
    public IMU getImu(){return imu;}
    public OpMode getOpMode(){return opMode;}
    public PIDController getHeadingPID() {
        return poseXYPID;
    }
    public PIDController getVelocityPID() {
        return speedPID;
    }

    //PRIVATE METHODS
    //Utility
    public void setMotorSpeeds(double fr, double fl, double rr, double rl){
        driveMotors.setPowers(new double[]{fr, -fl, rr, -rl});
    }

    public void setPoseXYPID(double p, double i, double d){poseXYPID.setPID(p,i,d);}
    public void setSpeedPID(double p, double i, double d){speedPID.setPID(p,i,d);}
    public void setTrajectoryPID(double p, double i, double d){
        trajectoryPID.setPID(p,i,d);
    }
    public void setPoseAnglePID(double p, double i, double d){
        poseAnglePID.setPID(p,i,d);
    }

    public static double[] calculateWheelSpeedsTurning(double degrees, double speed, double turnSpeed)
    {
        //Returns the speeds the motors need to move at to move. A negative turn speed turns right, a positive left.

        /*Wheel speed is calculated by getting the cosine wave (which we found matches the speed that
         * the wheels need to go) with a positive 45 or negative 45 shift, depending on the wheel. This works
         * so that no matter the degrees, it will always come out with the right value. A turn offset is added
         * to the end for corkscrewing, or turning while driving*/
        double strafeBoost = (Math.abs(0.5*Math.sin(Math.toRadians(degrees)))+1) ;
        double FRP = (-Math.cos(Math.toRadians(degrees + 45))*strafeBoost) * speed + turnSpeed;
        double FLP = (Math.cos(Math.toRadians(degrees - 45))*strafeBoost) * speed + turnSpeed;
        double RRP = (-Math.cos(Math.toRadians(degrees - 45))*strafeBoost) * speed + turnSpeed;
        double RLP = (Math.cos(Math.toRadians(degrees + 45))*strafeBoost) * speed + turnSpeed;

        double[] vals = {FRP, FLP, RRP, RLP};
        return vals;
    }

    public double[] getEncoderTicks(){
        double[] returnVal = new double[4];
        returnVal[0] = driveMotors.getMotorPositions()[0];
        returnVal[1] = driveMotors.getMotorPositions()[1];
        returnVal[2] = driveMotors.getMotorPositions()[2];
        returnVal[3] = driveMotors.getMotorPositions()[3];
        return returnVal;
    }


}
