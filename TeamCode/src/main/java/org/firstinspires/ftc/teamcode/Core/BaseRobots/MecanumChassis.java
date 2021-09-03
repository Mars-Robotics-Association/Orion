package org.firstinspires.ftc.teamcode.Core.BaseRobots;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Core.PIDController;

//Class for controlling the chassis of the demobot. Includes basic turning and driving. NOTE: the
//turn and move stuff here has to be called continuously (every loop), or it won't function properly.

//REQUIRED TO COMPILE: Phones | REV Hub
//REQUIRED TO RUN: Chassis

public class MecanumChassis
{
    ////Dependencies////
    private OpMode CurrentOpMode;
    private PIDController headingPIDController;
    private PIDController speedPID;
    private PIDController directionPID;
    private IMU imu;

    ////Variables////
    //Motors
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor RR;
    private DcMotor RL;
    private boolean rightReversed = false;
    private boolean leftReversed = false;

    //Brake pos
    private int FRBrakePos = 0;
    private int FLBrakePos = 0;
    private int RRBrakePos = 0;
    private int RLBrakePos = 0;
    //pid movement

    FtcDashboard dashboard;

    //telemetry
    private Telemetry RobotTelemetry;

    public PIDController GetHeadingPID() {
        return headingPIDController;
    }
    public PIDController GetVelocityPID() {
        return speedPID;
    }

    //Initializer
    public MecanumChassis(IMU setImu, DcMotor fr, DcMotor fl, DcMotor rr, DcMotor rl, Telemetry telemetry, boolean reveseRight, boolean reverseLeft){
        imu = setImu;
        FR = fr;
        FL = fl;
        RR = rr;
        RL = rl;
        RobotTelemetry = telemetry;
        rightReversed = reveseRight;
        leftReversed = reverseLeft;
    }

    ////STARTUP////
    public void Init(){
        SetMotorSpeeds(0,0,0,0);
        StopAndResetEncoders();
        SetModeRunWithoutEncoders();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        headingPIDController = new PIDController(0,0,0);//Create the pid controller.
        speedPID = new PIDController(0,0,0);//Create the pid controller.
        directionPID = new PIDController(0,0,0);//Create the pid controller.
    }

    ////CALLABLE METHODS////
    //Movement
    public void MoveAtAngle(double angle, double speed, double turnSpeed){
        //Tells robot to raw move at any angle. Turn speed variable causes it to sweep turn / corkscrew.
        //This is called continuously while the robot is driving.

        //Sets the mode so that robot can drive and record encoder values
        SetModeRunWithoutEncoders();

        //HEADING PID//
        //Uses pid controller to correct for heading error using (currentAngle, targetAngle)
        double headingPIDOffset = headingPIDController.getOutput(turnSpeed, imu.GetAngularVelocity());
        //if the number is not real, reset pid controller
        if(!(headingPIDOffset > 0 || headingPIDOffset <= 0)){
            headingPIDController.reset();
        }

        //TRANSLATIONAL PID//
        double velocityMag = 0; //TODO: fill out
        double velocityDir = 0; //TODO: fill out
        double magPIDOffset = speedPID.getOutput(speed, velocityMag);
        double dirPIDOffset = directionPID.getOutput(angle, velocityDir);


        RobotTelemetry.addData("Angular Velocity ", imu.GetAngularVelocity());
        RobotTelemetry.addData("Angular PID Offset ", headingPIDOffset);
        RobotTelemetry.addData("Velo X ", imu.GetVelocity().xVeloc + " m/s");
        RobotTelemetry.addData("Velo Y ", imu.GetVelocity().yVeloc + " m/s");
        RobotTelemetry.addData("Velo Z ", imu.GetVelocity().zVeloc + " m/s");

        //set the powers of the motors with pid offset applied
        //TODO remove line below
        headingPIDOffset *= -1;

        //Gets speeds for the motors
        double[] speeds = CalculateWheelSpeedsTurning(angle, speed, turnSpeed+headingPIDOffset);
        //SetMotorSpeeds(speeds[0]+headingPIDOffset, speeds[1]+headingPIDOffset, speeds[2]+headingPIDOffset, speeds[3]+headingPIDOffset);
        SetMotorSpeeds(speeds[0], speeds[1], speeds[2], speeds[3]);

        //Updates brake pos, as this is called continuously as robot is driving
        UpdateEncoderBrakePos();
    }
    public void SpotTurn(double speed)
    {
        //Turns the robot on the spot. Must be called continuously to work.
        // Positive speed turns left, negative right.

        //Use motors and record encoder values
        SetModeRunWithoutEncoders();

        //Set motor speeds all equal, as this causes it to do a spot turn
        SetMotorSpeeds(speed, speed, speed, speed);

        //Update the values for breaking
        UpdateEncoderBrakePos();
    }

    //Utility
    public void StopAndResetEncoders(){
        //Stops motors and resets encoders to 0
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void SetModeRunUsingEncoders(){
        //Sets motors to try and hold a velocity
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void SetModeRunWithoutEncoders(){
        //Sets motors to run like dc motors but record the encoder values
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void SetModeGoToEncoderPos(){
        //Tells motors to move to the target encoder values set in SetTargetEncoderPos()
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void SetTargetEncoderPos(int FRPos, int FLPos, int RRPos, int RLPos){
        //Sets the target encoder values on the drive motors. Motors will try to go there when SetModeGoToEncoderPos() is called
        FR.setTargetPosition(FRPos);
        FL.setTargetPosition(FLPos);
        RR.setTargetPosition(RRPos);
        RL.setTargetPosition(RLPos);
    }
    public void SetMotorSpeeds(double FRSpeed, double FLSpeed, double RRSpeed, double RLSpeed){
        //Sets the speeds of the motors
        double fr = FRSpeed;
        double fl = FLSpeed;
        double rr = RRSpeed;
        double rl = RLSpeed;

        if(rightReversed) {
            fr *= -1;
            rr *= -1;
        }
        if(leftReversed) {
            fl *= -1;
            rl *= -1;
        }

        FR.setPower(fr);
        FL.setPower(fl);
        RR.setPower(rr);
        RL.setPower(rl);
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
        SetTargetEncoderPos(FRBrakePos, FLBrakePos, RRBrakePos, RLBrakePos);
        SetModeGoToEncoderPos();
        SetMotorSpeeds(0.5,0.5,0.5,0.5);
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
