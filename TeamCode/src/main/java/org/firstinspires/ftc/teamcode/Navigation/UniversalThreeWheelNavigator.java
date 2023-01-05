package org.firstinspires.ftc.teamcode.Navigation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.EncoderArray;
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
@Config
public class UniversalThreeWheelNavigator
{
    ////DEPENDENCIES////
    private OpMode opMode;
    private MecanumChassis chassis;
    public MecanumChassis getChassis(){return chassis;}
    private HolonomicOdometry odometry;
    private EncoderArray encoders;

    ////CONFIGURABLE////
    protected static double[] encoderMultipliers = {-1,-1,-1}; //left right horizontal
    protected static double trackwidth = 10.8;
    protected static double centerWheelOffset = -6.8;

    protected static double minSpeed = 0.2; //the min speed to move if not at the target location or rotation
    protected static double moveCoefficient = 0.1; //how aggressively to move
    protected static double moveSmoothCoefficient = 0.1; //how much to ramp movement into its final speed
    protected static double turnCoefficient = 0.1; //how aggressively to turn
    protected static double turnSmoothCoefficient = 0.1; //how much to ramp turning into its final speed


    protected static double stopDistance = 0.2; //inches away for robot to stop
    protected static double stopDegrees = 2; //degrees away for robot to stop
    protected static double stopTime = 0.05; //how long it needs to be below speed threshold

    ////INTERNAL////
    double lastTimeAboveStopThreshold = 0;
    double controllerOffsetDegrees = 0;
    Pose2d targetPose;
    double lastMoveSpeed = 0;
    double lastTurnSpeed = 0;

    //---------------------------------------INIT CLASS-------------------------------------------//

    public void InitializeNavigator(OpMode setOpMode, BaseRobot baseRobot){
        opMode = setOpMode;
        chassis = new MecanumChassis(setOpMode, baseRobot.getChassisProfile(), baseRobot.getLog(), baseRobot);
        odometry = new HolonomicOdometry(trackwidth,centerWheelOffset);

        //get the drive motors in order (LEFT, RIGHT, HORIZONTAL) encoder
        DcMotor[] driveMotors = new DcMotor[]{
                chassis.driveMotors.getMotors()[0],
                chassis.driveMotors.getMotors()[1],
                chassis.driveMotors.getMotors()[2]};
        //creates the encoder array
        encoders = new EncoderArray(
                new DCMotorArray(driveMotors,new double[]{1,1,1},true),
                encoderMultipliers, 8192, 0.25);
        //initialize pose objects
        targetPose = new Pose2d();
    }

    public void update(){
        odometry.update(getDeadWheelPositions()[0], getDeadWheelPositions()[1], getDeadWheelPositions()[2]);
    }

    //-------------------------------------NAV FUNCTIONS------------------------------------------//

    public boolean turnTowards(double targetAngle, double speed){
        targetPose.setAngle(targetAngle);

        //get errors
        double turnError = calculateTurnError(targetAngle);

        //calculate speeds
        double turnSpeed = calculateTurnSpeed(turnError, minSpeed, speed);

        //turns
        chassis.rawTurn(turnSpeed);

        //prints telemetry
        opMode.telemetry.addData("GOING TO ANGLE:", targetAngle);
        opMode.telemetry.addData("TURN ERROR:", turnError);
        opMode.telemetry.addData("TURN SPEED:", turnSpeed);

        //return true if robot should keep driving, false if movement is done
        return !(shouldStop(0,turnError));
    }
    public boolean goTowardsPose(double targetX, double targetY, double targetAngle, double speed){
        targetPose.setPose(targetX,targetY,targetAngle);

        //get errors
        double[] moveDistanceAngleError = calculateMoveError(targetX,targetY);
        double turnError = calculateTurnError(targetAngle);

        //calculate speeds
        double moveSpeed = calculateScalarMoveSpeed(moveDistanceAngleError[0], minSpeed, speed);
        double turnSpeed = calculateTurnSpeed(turnError, minSpeed, speed);

        //drives
        chassis.rawDrive(moveDistanceAngleError[1], moveSpeed, turnSpeed);

        //prints telemetry
        opMode.telemetry.addData("GOING TO POSE:", "("+targetX+", "+targetY+", "+targetAngle+")");
        opMode.telemetry.addData("MOVE ERROR:", moveDistanceAngleError[0]);
        opMode.telemetry.addData("MOVE SPEED:", moveSpeed);
        opMode.telemetry.addData("MOVE ANGLE:", moveDistanceAngleError[1]);
        opMode.telemetry.addData("TURN ERROR:", turnError);
        opMode.telemetry.addData("TURN SPEED:", turnSpeed);

        //return true if robot should keep driving, false if movement is done
        return !(shouldStop(moveDistanceAngleError[0],turnError));
    }
    public boolean goTowardsPose(double targetX, double targetY, double targetAngle, double speed, ControllerInput controllerInput, double controllerWeight){
        opMode.telemetry.addData("GOING TO POSE:", "("+targetX+", "+targetY+", "+targetAngle+")");
        targetPose.setPose(targetX,targetY,targetAngle);
        double[] controllerAngleSpeedTurn = calculateControllerInputOffsets(controllerInput, controllerWeight);

        //get errors
        double[] moveDistanceAngleError = calculateMoveError(targetX,targetY);
        double turnError = calculateTurnError(targetAngle);

        //calculate speeds
        double moveSpeed = calculateScalarMoveSpeed(moveDistanceAngleError[0], minSpeed, speed)+controllerAngleSpeedTurn[1];
        double turnSpeed = calculateTurnSpeed(turnError, minSpeed, speed)+controllerAngleSpeedTurn[2];

        //prints telemetry
        opMode.telemetry.addData("GOING TO POSE:", "("+targetX+", "+targetY+", "+targetAngle+")");
        opMode.telemetry.addData("MOVE ERROR:", moveDistanceAngleError[0]);
        opMode.telemetry.addData("MOVE SPEED:", moveSpeed);
        opMode.telemetry.addData("MOVE ANGLE:", moveDistanceAngleError[1]);
        opMode.telemetry.addData("TURN ERROR:", turnError);
        opMode.telemetry.addData("TURN SPEED:", turnSpeed);

        //drives
        chassis.rawDrive(moveDistanceAngleError[1]+controllerAngleSpeedTurn[0], moveSpeed, turnSpeed);

        //return true if robot should keep driving, false if movement is done
        return !(shouldStop(moveDistanceAngleError[0],turnError));

    }

    public void setControllerOffset(double degrees){
        controllerOffsetDegrees = degrees;
    }

    //-----------------------------------INTERNAL FUNCTIONS---------------------------------------//

    //calculates the offsets based off of a gamepad
    protected double[] calculateControllerInputOffsets(ControllerInput controllerInput, double weight){
        double[] angleSpeedTurn = {0,0,0};
        angleSpeedTurn = new double[]{ //drives at (angle, speed, turnOffset)
                controllerInput.calculateLJSAngle() + controllerOffsetDegrees * weight,
                controllerInput.calculateLJSMag() * weight * chassis.profile.moveSpeed() * weight,
                controllerInput.getRJSX() * weight * chassis.profile.turnSpeed() * weight
        };
        return angleSpeedTurn;
    }

    protected double calculateTurnError(double targetAngle){
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

        //print telemetry
        opMode.telemetry.addData("ANGULAR ERROR: ", targetAngle-actualAngle);
        opMode.telemetry.addData("Corrected target angle: ", targetAngle);
        opMode.telemetry.addData("Corrected actual angle: ", actualAngle);

        //returns error
        return targetAngle-actualAngle;
    }

    protected double[] calculateMoveError(double targetX, double targetY){
        //get robot pose
        double actualX = getMeasuredPose().getX();
        double actualY = getMeasuredPose().getY();
        //calculates distance to target
        double distanceError = getDistance(targetX, targetY, actualX, actualY);
        //calculate the move angle
        double moveAngleError = getMeasuredPose().getHeading() + Math.toDegrees(Math.atan2(-(targetY-actualY), -(targetX-actualX)));

        //prints telemetry
        opMode.telemetry.addData("Target position: ", targetX + ", " +targetY);
        opMode.telemetry.addData("Actual position: ", actualX+", "+actualY);
        opMode.telemetry.addData("Distance error ", distanceError);
        opMode.telemetry.addData("Move angle error ", moveAngleError);

        //returns distance and move angle errors
        return new double[] {distanceError,moveAngleError};
    }

    protected double calculateScalarMoveSpeed(double error, double minSpeed, double maxSpeed){
        double finalSpeed = moveCoefficient*error*maxSpeed; //calculate base final speed based off proportional coefficient
        Math.max(minSpeed, Math.min(maxSpeed, finalSpeed)); //clamp speed between max and min
        if(Math.abs(error)<Math.abs(stopDistance)) finalSpeed = 0; //if within stop area, set speed to zero

        if(lastMoveSpeed<(finalSpeed-0.05)) finalSpeed = finalSpeed*moveSmoothCoefficient + lastMoveSpeed; //ramps up speed when target speed is increasing- this smooths out movement
        lastMoveSpeed = finalSpeed;

        return finalSpeed;
    }

    protected double calculateTurnSpeed(double error, double minSpeed, double maxSpeed){
        double finalSpeed = turnCoefficient*error*maxSpeed; //calculate base final speed based off proportional coefficient
        Math.max(minSpeed, Math.min(maxSpeed, finalSpeed)); //clamp speed between max and min
        if(Math.abs(error)>Math.abs(stopDegrees)) finalSpeed = 0; //if within stop area, set speed to zero

        if(lastTurnSpeed<(finalSpeed-0.05)) finalSpeed = finalSpeed*turnSmoothCoefficient + lastTurnSpeed; //ramps up speed when target speed is increasing- this smooths out movement
        lastTurnSpeed = finalSpeed;

        return finalSpeed;
    }

    //------------------------------------UTILITY FUNCTIONS---------------------------------------//

    protected double getDistance(double x1, double y1, double x2, double y2){
        double xError = x1-x2;
        double yError = y1-y2;
        return Math.sqrt((xError*xError)+(yError*yError)); //return distance
    }

    //returns true if value within threshold for given amount of time
    protected boolean shouldStop(double distanceError, double turnError) {
        //if far enough away, reset countdown
        if(Math.abs(distanceError) > stopDistance || Math.abs(turnError) > stopDegrees) lastTimeAboveStopThreshold = opMode.getRuntime();
        //if close enough
        else{
            //and countdown has run out
            if(opMode.getRuntime() - lastTimeAboveStopThreshold > stopTime)return true;
        }
        return false;
    }

    //fix target angle to bounds within -180 and 180
    protected double fixAngle(double angle){
        if(angle > 180) angle += -360;
        else if(angle < -180) angle += 360;
        return angle;
    }


    //------------------------------------GETTERS & SETTERS---------------------------------------//

    public double[] getDeadWheelPositions(){return encoders.getPositions();}
    public void setMeasuredPose(double x, double y, double angle){odometry.updatePose(new Pose2d(x,y,angle));}
    public Pose2d getMeasuredPose(){return odometry.getPose();}
    public Pose2d getTargetPose(){return targetPose;}
    public double getRobotAngleDegrees(){return Math.toDegrees(odometry.getPose().getHeading());}

}
