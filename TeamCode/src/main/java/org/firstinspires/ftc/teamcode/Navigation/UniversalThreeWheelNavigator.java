package org.firstinspires.ftc.teamcode.Navigation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Core.HermesLog.HermesLog;
import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.EncoderArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.PIDController;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.Navigation.Odometry.geometry.Pose2d;

/*
=====EXAMPLE=====
DO
NOT
CHANGE
THIS
FILE
FOR
SPECIFIC
ROBOTS
*/

//A basic navigation class with odometry functions
public class UniversalThreeWheelNavigator
{
    ////DEPENDENCIES////
    private OpMode opMode;
    private MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}
    private HolonomicOdometry odometry;
    private EncoderArray encoders;
    private DistanceSensor distancePort;
    private DistanceSensor distanceStarboard;
    private ColorSensor colorSensor;

    ////CONFIGURABLE////

    protected static double[] encoderMultipliers = {-1,-1,-1}; //left right horizontal
    protected static double trackwidth = 10.8;
    protected static double centerWheelOffset = -6.8;

    protected static double turnPID_P = 0.035;
    protected static double turnPID_I = 0;
    protected static double turnPID_D = -0.1;

    protected static double movePID_D = 0.1;
    protected static double movePID_I = 0;
    protected static double movePID_P = 0.3;

    protected static double stopSpeedThreshold = 0.1; //how slow the robot needs to be moving before it stops
    protected static double stopTimeThreshold = 0.2; //how long it needs to be below speed threshold

    ////INTERNAL////
    PIDController turningPID;
    PIDController movePID;

    double lastTimeAboveStopThreshold = 0;
    double controllerOffsetDegrees = 0;
    Pose2d targetPose;

    public void InitializeNavigator(OpMode setOpMode, BaseRobot baseRobot, DistanceSensor setDistancePort, DistanceSensor setDistanceStarboard, ColorSensor setColorSensor){
        opMode = setOpMode;
        chassis = new MecanumChassis(setOpMode, baseRobot.getChassisProfile(), new HermesLog("Demobot", 200, setOpMode), baseRobot);
        odometry = new HolonomicOdometry(trackwidth,centerWheelOffset);
        distancePort = setDistancePort;
        distanceStarboard = setDistanceStarboard;
        colorSensor = setColorSensor;

        //get the drive motors in order (LEFT, RIGHT, HORIZONTAL) encoder
        DcMotor[] driveMotors = new DcMotor[]{
                chassis.driveMotors.getMotors()[0],
                chassis.driveMotors.getMotors()[1],
                chassis.driveMotors.getMotors()[2]};

        //creates the encoder array
        encoders = new EncoderArray(
                new DCMotorArray(driveMotors,new double[]{1,1,1},true),
                encoderMultipliers, 8192, 0.25);

        //set PIDs
        turningPID = new PIDController(turnPID_P, turnPID_I, turnPID_D);
        resetTurnPID();
        movePID = new PIDController(movePID_P, movePID_I, movePID_D);
        resetMovePID();

        //initialize pose objects
        targetPose = new Pose2d();
    }

    public void update(){
        odometry.update(getDeadWheelPositions()[0], getDeadWheelPositions()[1], getDeadWheelPositions()[2]);
        setTurnPID(turnPID_P, turnPID_I, turnPID_D);
        setMovePID(movePID_P, movePID_I, movePID_D);
    }

    ////NAVIGATION FUNCTIONS////

    //turns towards the given angle. Returns zero when pid is within certain threshold
    public boolean turnTowards(double targetAngle, double speed, double overrideStopSpeedThreshold, double overrideStopTimeThreshold){
        double turnSpeed = calculateTurnSpeed(targetAngle,speed);

        //turn the robot
        chassis.rawTurn(turnSpeed);

        return checkIfShouldStop(overrideStopSpeedThreshold, overrideStopTimeThreshold, turnSpeed);
    }
    public boolean turnTowards(double targetAngle, double speed){return turnTowards(targetAngle, speed, stopSpeedThreshold, stopTimeThreshold);}

    //turns towards the given angle. Returns zero when pid is within certain threshold
    public boolean moveTowards(double targetX, double targetY, double speed, double overrideStopSpeedThreshold, double overrideStopTimeThreshold){
        double[] moveAngleSpeed = calculateMoveAngleSpeed(targetX,targetY,speed);
        double moveAngle = moveAngleSpeed[0];
        double moveSpeed = moveAngleSpeed[1];

        //drives
        chassis.rawDrive(moveAngle, moveSpeed, 0);
        //checks if it should stop
        return checkIfShouldStop(overrideStopSpeedThreshold, overrideStopTimeThreshold, moveSpeed);
    }
    public boolean moveTowards(double targetX, double targetY, double speed){return moveTowards(targetX, targetY, speed, stopSpeedThreshold, stopTimeThreshold);}

    //turns towards the given angle. Returns zero when pid is within certain threshold
    public boolean goTowardsPose(double targetX, double targetY, double targetAngle, double speed, double overrideStopSpeedThreshold, double overrideStopTimeThreshold){
        opMode.telemetry.addData("GOING TO POSE:", "("+targetX+", "+targetY+", "+targetAngle+")");
        targetPose.setPose(targetX,targetY,targetAngle);

        //calculate speeds
        double[] moveAngleSpeed = calculateMoveAngleSpeed(targetX,targetY,speed);
        double moveAngle = moveAngleSpeed[0];
        double moveSpeed = moveAngleSpeed[1];
        double turnSpeed = calculateTurnSpeed(targetAngle,speed);

        //drives
        chassis.rawDrive(moveAngle, moveSpeed, turnSpeed);
        //if both speeds are very low, stop
        if(checkIfShouldStop(overrideStopSpeedThreshold, overrideStopTimeThreshold, moveSpeed) &&
                checkIfShouldStop(overrideStopSpeedThreshold, overrideStopTimeThreshold, turnSpeed))
            return true;
        else
            return false;
    }
    public boolean goTowardsPose(double targetX, double targetY, double targetAngle, double speed){return goTowardsPose(targetX, targetY, targetAngle, speed, stopSpeedThreshold, stopTimeThreshold);}

    public boolean goTowardsPose(double targetX, double targetY, double targetAngle, double speed, ControllerInput controllerInput, double controllerWeight){
        opMode.telemetry.addData("GOING TO POSE:", "("+targetX+", "+targetY+", "+targetAngle+")");

        //calculate speeds
        double[] moveAngleSpeed = calculateMoveAngleSpeed(targetX,targetY,speed);
        double[] controllerAngleSpeedTurn = calculateControllerInputAngleSpeedTurn(controllerInput, controllerWeight);
        double moveAngle = moveAngleSpeed[0] + controllerAngleSpeedTurn[0];
        double moveSpeed = moveAngleSpeed[1] + controllerAngleSpeedTurn[1];
        double turnSpeed = calculateTurnSpeed(targetAngle,speed) + controllerAngleSpeedTurn[2];

        //drives
        chassis.rawDrive(moveAngle, moveSpeed, turnSpeed);
        //if both speeds are very low, stop
        if(checkIfShouldStop(stopSpeedThreshold, stopTimeThreshold, moveSpeed) &&
                checkIfShouldStop(stopSpeedThreshold, stopTimeThreshold, turnSpeed))
            return true;
        else
            return false;
    }

    public double[] calculateControllerInputAngleSpeedTurn(ControllerInput controllerInput, double speedMultiplier){
        double[] angleSpeedTurn = {0,0,0};
        angleSpeedTurn = new double[]{ //drives at (angle, speed, turnOffset)
                controllerInput.calculateLJSAngle() + controllerOffsetDegrees * speedMultiplier,
                controllerInput.calculateLJSMag() * speedMultiplier * chassis.profile.moveSpeed() * speedMultiplier,
                controllerInput.getRJSX() * speedMultiplier * chassis.profile.turnSpeed() * speedMultiplier
        };
        return angleSpeedTurn;
    }
    public void setControllerOffset(double degrees){
        controllerOffsetDegrees = degrees;
    }

    ////INTERNAL////

    //angle is in degrees
    protected double calculateTurnSpeed(double targetAngle, double speed){
        targetPose.setAngle(targetAngle);
        double actualAngle = getRobotAngleDegrees();
        opMode.telemetry.addData("Initial target angle: ", targetAngle);
        opMode.telemetry.addData("Initial actual angle: ", actualAngle);

        //fix target angle to bounds within -180 and 180
        fixAngle(targetAngle);

        //fix delta angle if delta is greater than 180 degrees (so turn the opposite direction)
        boolean fixedDelta = false;
        //if the error is greater than 180
        if(Math.abs(targetAngle-actualAngle) > 180){
            //add 180 degrees to everything and fix the angles
            targetAngle += 180;
            actualAngle += 180;
            targetAngle = fixAngle(targetAngle);
            actualAngle = fixAngle(actualAngle);
            fixedDelta = true;
        }
        opMode.telemetry.addData("Fixed delta: ", fixedDelta);

        //calculate turn speed based off of PID
        double turnSpeed = (speed * turningPID.getOutput(actualAngle, targetAngle));
        turnSpeed += (0.2*Math.signum(turnSpeed));
        //print telemetry
        opMode.telemetry.addData("ERROR: ", targetAngle-actualAngle);
        opMode.telemetry.addData("Corrected target angle: ", targetAngle);
        opMode.telemetry.addData("Corrected actual angle: ", actualAngle);
        opMode.telemetry.addData("Turn speed ", turnSpeed);
        return turnSpeed;
    }

    protected double[] calculateMoveAngleSpeed(double targetX, double targetY, double speed){
        targetPose.setXY(targetX,targetY);
        //get robot pose
        double actualX = getMeasuredPose().getX();
        double actualY = getMeasuredPose().getY();
        //calculates distance to target
        double distanceError = getDistance(targetX, targetY, actualX, actualY);
        //calculates speed based off of distance from target
        double moveSpeed = speed * movePID.getOutput(distanceError, 0);
        moveSpeed += (0.2*Math.signum(moveSpeed));
        //calculate the move angle
        double moveAngle = getMeasuredPose().getHeading() + Math.toDegrees(Math.atan2(-(targetY-actualY), -(targetX-actualX)));

        //prints telemetry
        opMode.telemetry.addData("Target position: ", targetX + ", " +targetY);
        opMode.telemetry.addData("Actual position: ", actualX+", "+actualY);
        opMode.telemetry.addData("Distance error ", distanceError);
        opMode.telemetry.addData("Move speed ", moveSpeed);
        opMode.telemetry.addData("Move angle ", moveAngle);

        return new double[] {moveAngle,moveSpeed};
    }

    ////UTILITY////
    private double getDistance(double x1, double y1, double x2, double y2){
        double xError = x1-x2;
        double yError = y1-y2;
        return Math.sqrt((xError*xError)+(yError*yError)); //return distance
    }

    //returns true if has been going below speed threshold for a long enough amount of time
    private boolean checkIfShouldStop(double overrideStopSpeedThreshold, double overrideStopTimeThreshold, double speed) {
        //if going fast enough, reset clock
        if(Math.abs(speed) > overrideStopSpeedThreshold) lastTimeAboveStopThreshold = opMode.getRuntime();
        //if going slow enough
        else{
            //and if its been a long enough time since went fast, return true
            if(opMode.getRuntime() - lastTimeAboveStopThreshold > overrideStopTimeThreshold){
                return true;
            }
        }
        //return false if not at destination
        return false;
    }

    //fix target angle to bounds within -180 and 180
    public double fixAngle(double angle){
        if(angle > 180) angle += -360;
        else if(angle < -180) angle += 360;
        return angle;
    }

    //gets the positions of the dead wheels
    public double[] getDeadWheelPositions(){return encoders.getPositions();}

    public void setMeasuredPose(double x, double y, double angle){odometry.updatePose(new Pose2d(x,y,angle));}
    public Pose2d getMeasuredPose(){return odometry.getPose();}
    public Pose2d getTargetPose(){return targetPose;}
    public double getRobotAngleDegrees(){return Math.toDegrees(odometry.getPose().getHeading());}

    //sets PIDs
    public void setTurnPID(double p, double i, double d){turningPID.setPID(p,i,d);}
    public void resetTurnPID(){turningPID.reset();}
    public void setMovePID(double p, double i, double d){movePID.setPID(p,i,d);}
    public void resetMovePID(){movePID.reset();}

}
