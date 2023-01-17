package org.firstinspires.ftc.teamcode.Navigation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Core.InputSystem.ControllerInput;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.EncoderArray;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Chassis.ChassisProfile;
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
    public static double[] encoderMultipliers = {-1,-1,-1}; //left right horizontal
    public static double trackwidth = 10.8;
    public static double centerWheelOffset = -6.8;

    public static double minSpeed = 0.1; //the min speed to move if not at the target location or rotation
    public static double moveCoefficient = 0.1; //how aggressively to move
    public static double moveSmoothCoefficient = 0.1; //how much to ramp movement into its final speed
    public static double turnCoefficient = 0.01; //how aggressively to turn
    public static double turnSmoothCoefficient = 0.1; //how much to ramp turning into its final speed

    public static double slowDistance = 4; //when to start slowing down
    public static double slowDegrees = 30; //when to start slowing down
    public static double stopDistance = 0.2; //inches away for robot to stop
    public static double stopDegrees = 2; //degrees away for robot to stop
    public static double stopTime = 0.05; //how long it needs to be below speed threshold

    ////INTERNAL////
    double lastTimeAboveStopThreshold = 0;
    double controllerOffsetDegrees = 0;
    double[] targetPose;
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
        targetPose = new double[]{0,0,0};
    }

    public void update(){
        odometry.update(getDeadWheelPositions()[0], getDeadWheelPositions()[1], getDeadWheelPositions()[2]);
    }

    //-------------------------------------NAV FUNCTIONS------------------------------------------//

    //a function that drives solely in headless mode and locks movement and turning to global axis
    public void driveWithGamepadAbsolute(ControllerInput controllerInput, double speed) {
        ChassisProfile chassisProfile = chassis.getProfile();

        //use absolute drive turn offset if and only if joystick magnitude is big
        double turnSpeed = 0;
        if(controllerInput.calculateRJSMag() > 0.1){
            //gets target angle from joystick
            double targetAngle = controllerInput.calculateRJSAngle();
            //get error
            double turnError = calculateTurnError(targetAngle, chassis.getImu().getRobotAngle());
            //calculate speeds
            turnSpeed = -calculateTurnSpeed(turnError, minSpeed, speed, false)*chassisProfile.turnSpeed()*controllerInput.calculateRJSMag();
            //prints telemetry
            opMode.telemetry.addData("GOING TO ANGLE:", targetAngle);
            opMode.telemetry.addData("TURN ERROR:", turnError);
        }

        opMode.telemetry.addData("DRIVE TURN SPEED:", turnSpeed);

        //MOVE if left joystick magnitude > 0.1
        if (controllerInput.calculateLJSMag() > 0.1) {
            //initial values
            double driveAngle = controllerInput.calculateLJSAngle()+chassis.getInputOffset()+chassis.getImu().getRobotAngle();;
            double driveSpeed = controllerInput.calculateLJSMag() * chassisProfile.moveSpeed() * speed;

            chassis.rawDriveRamped(driveAngle, driveSpeed, turnSpeed, moveSmoothCoefficient);//drives at (angle, speed, turnOffset)
            opMode.telemetry.addData("Moving at ", controllerInput.calculateLJSAngle());
        }

        //TURN if right joystick magnitude > 0.1 and not moving
        else if (controllerInput.calculateRJSMag() > 0.2) chassis.rawTurn(turnSpeed);
        else chassis.stop();
    }

    public boolean turnTowards(double targetAngle, double speed){
        targetPose[2] = targetAngle;

        //get errors
        double turnError = calculateTurnError(targetAngle);

        //calculate speeds
        double turnSpeed = calculateTurnSpeed(turnError, minSpeed, speed)*chassis.getProfile().turnSpeed();

        //turns
        chassis.rawTurn(turnSpeed);

        //prints telemetry
        opMode.telemetry.addData("GOING TO ANGLE:", targetAngle);
        opMode.telemetry.addData("TURN ERROR:", turnError);
        opMode.telemetry.addData("TURN SPEED:", turnSpeed);

        //return true if robot should keep driving, false if movement is done
        return !(shouldStop(0,turnError));
    }
    public boolean moveTowards(double targetX, double targetY, double speed){
        targetPose[0] = targetX;
        targetPose[1] = targetY;

        //get errors
        double[] moveDistanceAngleError = calculateMoveError(targetX,targetY);

        //calculate speeds
        double moveSpeed = calculateScalarMoveSpeed(moveDistanceAngleError[0], minSpeed, speed)*chassis.getProfile().moveSpeed();

        //drives
        chassis.rawDrive(moveDistanceAngleError[1], moveSpeed, 0);

        //prints telemetry
        opMode.telemetry.addData("MOVE ERROR:", moveDistanceAngleError[0]);
        opMode.telemetry.addData("MOVE SPEED:", moveSpeed);
        opMode.telemetry.addData("MOVE ANGLE:", moveDistanceAngleError[1]);

        //return true if robot should keep driving, false if movement is done
        return !(shouldStop(moveDistanceAngleError[0],0));
    }
    public boolean goTowardsPose(double targetX, double targetY, double targetAngle, double speed){
        targetPose[0] = targetX;
        targetPose[1] = targetY;
        targetPose[2] = targetAngle;

        //get errors
        double[] moveDistanceAngleError = calculateMoveError(targetX,targetY);
        double turnError = calculateTurnError(targetAngle);

        //calculate speeds
        double moveSpeed = calculateScalarMoveSpeed(moveDistanceAngleError[0], minSpeed, speed)*chassis.getProfile().moveSpeed();
        double turnSpeed = calculateTurnSpeed(turnError, minSpeed, speed)*chassis.getProfile().turnSpeed();

        //drives
        chassis.rawDrive(moveDistanceAngleError[1], moveSpeed, turnSpeed);

        //prints telemetry
        //opMode.telemetry.addData("GOING TO POSE:", "("+targetX+", "+targetY+", "+targetAngle+")");
        opMode.telemetry.addData("GOING TO POSE:", "("+targetPose[0]+", "+targetPose[1]+", "+targetPose[2]+")");
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
        targetPose[0] = targetX;
        targetPose[1] = targetY;
        targetPose[2] = targetAngle;

        //get errors
        double[] moveDistanceAngleError = calculateMoveError(targetX,targetY);
        double turnError = calculateTurnError(targetAngle);

        //calculate speeds
        double moveSpeed = calculateScalarMoveSpeed(moveDistanceAngleError[0], minSpeed, speed)*chassis.getProfile().moveSpeed();
        double turnSpeed = calculateTurnSpeed(turnError, minSpeed, speed)*chassis.getProfile().turnSpeed();

        //prints telemetry
        opMode.telemetry.addData("GOING TO POSE:", "("+targetX+", "+targetY+", "+targetAngle+")");
        opMode.telemetry.addData("MOVE ERROR:", moveDistanceAngleError[0]);
        opMode.telemetry.addData("MOVE SPEED:", moveSpeed);
        opMode.telemetry.addData("MOVE ANGLE:", moveDistanceAngleError[1]);
        opMode.telemetry.addData("TURN ERROR:", turnError);
        opMode.telemetry.addData("TURN SPEED:", turnSpeed);

        //drives
        rawDriveWithControllerOffsets(controllerInput, controllerWeight, moveDistanceAngleError[1], moveSpeed, turnSpeed);
        chassis.rawDrive(moveDistanceAngleError[1], moveSpeed, turnSpeed);

        //return true if robot should keep driving, false if movement is done
        return !(shouldStop(moveDistanceAngleError[0],turnError));

    }

    //-----------------------------------INTERNAL FUNCTIONS---------------------------------------//
    //drives using controller offsets
    public void rawDriveWithControllerOffsets(ControllerInput controllerInput, double controllerWeight, double angle, double speed, double turnOffset){
        double[] inputOffsets = calculateControllerInputOffsets(controllerInput, controllerWeight);
        double finalAngle = 0;
        double finalSpeed = 0;
        double finalTurn = turnOffset + inputOffsets[2];

        //adds movement vectors
        double inputAngle = inputOffsets[0];
        double inputMag = inputOffsets[1];
        double x1 = speed*Math.cos(angle);
        double y1 = speed*Math.sin(angle);
        double x2 = inputMag*Math.cos(inputAngle);
        double y2 = inputMag*Math.sin(inputAngle);

        double dx = x1+x2;
        double dy = y1+y2;

        opMode.telemetry.addData("RAW DRIVE WITH CONTROLLER DATA:", "x2 "+x2+", "+"y2 "+y2+", "+"dx "+dx+", "+"dy "+dy);

        finalAngle = Math.atan2(dx,dy);
        finalSpeed = Math.sqrt(dx*dx + dy*dy);

        //finalSpeed = Math.sqrt(Math.pow(speed,2) + Math.pow(inputMag,2) + (2*speed*inputMag*Math.cos(inputAngle-angle)));
        //finalAngle = angle + Math.atan2(inputMag*Math.sin(inputAngle-angle), speed+(inputMag*Math.cos(inputAngle-angle)));

        chassis.rawDrive(finalAngle,finalSpeed,finalTurn);
    }

    //calculates the offsets based off of a gamepad
    public double[] calculateControllerInputOffsets(ControllerInput controllerInput, double weight){
        ChassisProfile chassisProfile = chassis.getProfile();
        //use absolute drive turn offset if and only if joystick magnitude is big
        double turnSpeed = 0;
        double driveAngle = 0;
        double driveSpeed = 0;
        if(controllerInput.calculateRJSMag() > 0.8){
            //gets target angle from joystick
            double targetAngle = controllerInput.calculateRJSAngle();
            //get error
            double turnError = calculateTurnError(targetAngle, chassis.getImu().getRobotAngle());
            //calculate speeds
            turnSpeed = calculateTurnSpeed(turnError, minSpeed, weight);
            //prints telemetry

        }
        if(controllerInput.calculateLJSMag() > 0.1){
            driveAngle = controllerInput.calculateLJSAngle()-180+chassis.getImu().getRobotAngle();
            driveSpeed = controllerInput.calculateLJSMag() * chassisProfile.moveSpeed() * weight;
        }
        opMode.telemetry.addData("GAMEPAD OFFSET ANGLE:", driveAngle);
        opMode.telemetry.addData("GAMEPAD OFFSET SPEED:", driveSpeed);
        opMode.telemetry.addData("GAMEPAD OFFSET TURN:", turnSpeed);


        return new double[]{driveAngle,driveSpeed,turnSpeed};
    }
//    //calculates the offsets based off of a gamepad
//    public double[] calculateControllerInputOffsets(ControllerInput controllerInput, double weight){
//        double[] angleSpeedTurn = {0,0,0};
//        angleSpeedTurn = new double[]{ //drives at (angle, speed, turnOffset)
//                controllerInput.calculateLJSAngle() + controllerOffsetDegrees * weight,
//                controllerInput.calculateLJSMag() * weight * chassis.profile.moveSpeed() * weight,
//                controllerInput.getRJSX() * weight * chassis.profile.turnSpeed() * weight
//        };
//        return angleSpeedTurn;
//    }

    public double calculateTurnError(double targetAngle){
        double actualAngle = getRobotAngleDegrees();
        return calculateTurnError(targetAngle, actualAngle);
    }
    public double calculateTurnError(double targetAngle, double actualAngle){
//        opMode.telemetry.addData("Initial target angle: ", targetAngle);
//        opMode.telemetry.addData("Initial actual angle: ", actualAngle);

        //fix target angle to bounds within -180 and 180
        targetAngle = fixAngle(targetAngle); //-40
        actualAngle = fixAngle(actualAngle); //80
//        opMode.telemetry.addData("Fix 1 target angle: ", targetAngle);
//        opMode.telemetry.addData("Fix 2 actual angle: ", actualAngle);
        //fix delta angle if delta is greater than 180 degrees (so turn the opposite direction)
        //if the error is greater than 180
        if(Math.abs(targetAngle-actualAngle) > 180){
            //add 180 degrees to everything and fix the angles
            targetAngle += 180;
            actualAngle += 180;
            targetAngle = fixAngle(targetAngle);
            actualAngle = fixAngle(actualAngle);
        }

        //print telemetry
        opMode.telemetry.addData("ANGULAR ERROR: ", targetAngle-actualAngle);
        opMode.telemetry.addData("Corrected target angle: ", targetAngle);
        opMode.telemetry.addData("Corrected actual angle: ", actualAngle);

        //returns error
        return (targetAngle-actualAngle);
    }

    public double[] calculateMoveError(double targetX, double targetY){
        //get robot pose
        double actualX = getMeasuredPose().getX();
        double actualY = getMeasuredPose().getY();
        //calculates distance to target
        double distanceError = getDistance(targetX, targetY, actualX, actualY);
        //calculate the move angle
        double moveAngleError = fixAngle(-Math.toDegrees(getMeasuredPose().getHeading()) + Math.toDegrees(Math.atan2(-(targetY-actualY), -(targetX-actualX))));
        //double moveAngleError = fixAngle(-Math.toDegrees(getMeasuredPose().getHeading()) + Math.toDegrees(Math.atan2((targetY-actualY), (targetX-actualX))));  // TODO: For Ingenuity

        //prints telemetry
        opMode.telemetry.addData("Target position: ", targetX + ", " +targetY);
        opMode.telemetry.addData("Actual position: ", actualX+", "+actualY);
        opMode.telemetry.addData("Distance error ", distanceError);
        opMode.telemetry.addData("Move angle error ", moveAngleError);

        //returns distance and move angle errors
        return new double[] {distanceError,moveAngleError};
    }

    public double calculateScalarMoveSpeed(double error, double minSpeed, double maxSpeed){
        double finalSpeed = moveCoefficient*error*maxSpeed; //calculate base final speed based off proportional coefficient
        if(Math.abs(error)<slowDistance) finalSpeed = finalSpeed*(error/slowDistance); //slows when gets close to target

        finalSpeed = Math.max(minSpeed, Math.min(maxSpeed, finalSpeed)); //clamp speed between max and min
        if(Math.abs(error)<Math.abs(stopDistance)) finalSpeed = 0; //if within stop area, set speed to zero

        if(lastMoveSpeed<(finalSpeed-0.05)&&!(Math.abs(error)<slowDistance)) finalSpeed = finalSpeed*moveSmoothCoefficient + lastMoveSpeed; //ramps up speed when target speed is increasing- this smooths out movement
        lastMoveSpeed = finalSpeed;

        return finalSpeed;
    }

    public double calculateTurnSpeed(double error, double minSpeed, double maxSpeed){
        return calculateTurnSpeed(error,minSpeed,maxSpeed,true);
    }
    public double calculateTurnSpeed(double error, double minSpeed, double maxSpeed, boolean stopWithinRange){
        double finalSpeed = turnCoefficient*error*maxSpeed; //calculate base final speed based off proportional coefficient
        if(Math.abs(error)<slowDegrees) finalSpeed = finalSpeed*Math.abs((error/slowDegrees)); //slows when gets close to target

        //clamp speed between max and min in both positive and negative
        if(finalSpeed > 0) finalSpeed = Math.max(minSpeed, Math.min(maxSpeed, finalSpeed));
        if(finalSpeed < 0) finalSpeed = Math.max(-maxSpeed, Math.min(-minSpeed, finalSpeed));

        if(Math.abs(error)<Math.abs(stopDegrees)&&stopWithinRange) finalSpeed = 0; //if within stop area, set speed to zero


        //ramps up speed when target speed is increasing- this smooths out movement
        if(Math.abs(lastTurnSpeed)<Math.abs(finalSpeed-0.05)&&!(Math.abs(error)<slowDegrees)) finalSpeed = finalSpeed*turnSmoothCoefficient + lastTurnSpeed;
        lastTurnSpeed = finalSpeed;

        return finalSpeed;
    }

    //------------------------------------UTILITY FUNCTIONS---------------------------------------//

    public double getDistance(double x1, double y1, double x2, double y2){
        double xError = x1-x2;
        double yError = y1-y2;
        return Math.sqrt((xError*xError)+(yError*yError)); //return distance
    }

    //returns true if value within threshold for given amount of time
    public boolean shouldStop(double distanceError, double turnError) {
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
    public double fixAngle(double angle){
        if(angle > 180) angle -= 360;
        else if(angle < -180) angle += 360;
        return angle;
    }


    //------------------------------------GETTERS & SETTERS---------------------------------------//

    public double[] getDeadWheelPositions(){return encoders.getPositions();}
    public void setMeasuredPose(double x, double y, double angle){odometry.updatePose(new Pose2d(x,y,angle));}
    public Pose2d getMeasuredPose(){return odometry.getPose();}
    public double[] getTargetPose(){return targetPose;}
    public double getRobotAngleDegrees(){return Math.toDegrees(odometry.getPose().getHeading());}

}
