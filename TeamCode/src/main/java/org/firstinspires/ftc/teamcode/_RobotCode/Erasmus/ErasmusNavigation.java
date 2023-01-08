package org.firstinspires.ftc.teamcode._RobotCode.Erasmus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.BaseRobot;
import org.firstinspires.ftc.teamcode.Navigation.UniversalThreeWheelNavigator;

@Config
class ErasmusNavigation extends UniversalThreeWheelNavigator
{
    ////Variables////
    public static double[] nav_encoderMultipliers = {1, -1, 1} ; //left right horizontal
    public static double nav_trackwidth = 7.4 ;
    public static double nav_centerWheelOffset = -4 ;  // Check for Ingy

    public static double nav_turnPID_P = 0.01 ;  // Default = 0.035
    public static double nav_turnPID_I = 0 ;  // Default = 0
    public static double nav_turnPID_D = 0.00 ;  // Default = -0.1

    public static double nav_movePID_P = 0.06 ;  // Default = 0.3
    public static double nav_movePID_I = 0 ;  // Default = 0
    public static double nav_movePID_D = 0.00 ;  // Default = 0.1

    public static double nav_stopSpeedThreshold = 0.10 ; //how slow the robot needs to be moving before it stops
    public static double nav_stopTimeThreshold = 0.2 ; //how long it needs to be below speed threshold
    public static double nav_stopDistanceThreshold = 0.1 ; //TODO: Added by EA
    private double lastTimeNotThere = 0 ; //TODO: Added by EA
    public static double nav_stopAngleThreshold = 2 ; //TODO: Added by EA
    public static double kProportional = 0.1 ; //TODO: Added by EA
    public static double minDriveSpeed = 0.15 ; //TODO: Added by EA
    public static double minTurnSpeed = 0.3 ; //TODO: Added by EA
    public static double rampSpeedIncrement = 0.04 ;
    private double lastSpeed = 0 ; // This is used to calculate ramp speed
    //private double lastDistance = 0 ;  // This is used to calculate if we stopped (stuck). The concept is very problematic.
    // TODO:  I think a better way is to have a time limit on the move to pose method (or the calling one), like in FLL.
    //        I also think this would be best implemented in the calling method (outside this class).
    private OpMode opMode ;

    public ErasmusNavigation(OpMode setOpMode, BaseRobot baseRobot, DistanceSensor setDistancePort, DistanceSensor setDistanceStarboard, ColorSensor setColorSensor) {
        ////CONFIGURABLE////
        encoderMultipliers = nav_encoderMultipliers;
        trackwidth = nav_trackwidth;
        centerWheelOffset = nav_centerWheelOffset;

        turnPID_P = nav_turnPID_P;
        turnPID_I = nav_turnPID_I;
        turnPID_D = nav_turnPID_D;

        movePID_D = nav_movePID_D;
        movePID_I = nav_movePID_I;
        movePID_P = nav_movePID_P;

        stopSpeedThreshold = nav_stopSpeedThreshold; //how slow the robot needs to be moving before it stops
        stopTimeThreshold = nav_stopTimeThreshold; //how long it needs to be below speed threshold
        //stopDistanceThreshold = nav_stopDistanceThreshold ;
        //opMode = setOpMode ;  // TODO: Delete when we remember how inheritance works with composition :)
        InitializeNavigator(setOpMode, baseRobot, setDistancePort, setDistanceStarboard, setColorSensor);
    }

    public boolean moveToPose( double targetX, double targetY, double targetHeading, double targetSpeed) {
        /* -----------------------------------------------------------------------------
        Main driving method.
        1. Calculates the vector needed to drive to target position (no rotation) - calculateVector()
        2. Determine if we are at the target position/pose (rotation).
        3. If still not there, calculate the drive speed - calculateScalarSpeed() - and
            the rotation - calculateTurnSpeed() - to reach the target pose.
        4. Send the drive instruction to the chassis - getChassis().rawDrive().
        --------------------------------------------------------------------------------- */
        // TODO: Write our own calculateTurnSpeed().
        opMode.telemetry.addData("GOING TO POSE:", "("+targetX+", "+targetY+", "+targetHeading+")");
        update() ;
        double[] driveVector = calculateVector(targetX, targetY) ;
        if (areWeThereYet(driveVector[0])) {
            // We are there. Stop driving/turning.
            lastSpeed = 0 ;
            //lastDistance = 0 ; // TODO: Delete or keep ???
            return true ;
        }
        getChassis().rawDrive( driveVector[1], calculateScalarSpeed(driveVector[0], targetSpeed), calculateTurnSpeed(targetHeading, targetSpeed) ) ;
        return false ; // Not there yet => Keep going...
    }

    public double[] calculateVector(double targetX, double targetY) {
        double xError = targetX-getMeasuredPose().getX() ;
        double yError = targetY-getMeasuredPose().getY() ;
        //  Distance to the target
        double scalarDistance = Math.sqrt((xError*xError)+(yError*yError));
        // Angle to the target, relative to the robot's heading
        double angle = getMeasuredPose().getHeading() + Math.toDegrees(Math.atan2(-yError, -xError));
        return new double[] { scalarDistance, angle } ;
    }

    public double calculateScalarSpeed( double distance, double maxSpeed ) {
        double finalSpeed = kProportional*distance*maxSpeed ;                           // Convert the distance error into a speed
        if (Math.abs(finalSpeed) > Math.abs(maxSpeed)) finalSpeed = maxSpeed ;          // Cap the speed to the maxSpeed provided
        else if (Math.abs(distance) < nav_stopDistanceThreshold) finalSpeed = 0 ;       // Stop driving if the robot is there (maybe waiting to finish rotating)
        else if (Math.abs(finalSpeed) < minDriveSpeed) finalSpeed = minDriveSpeed ;     // When close, make sure we're still driving enough to move

        if (lastSpeed < (finalSpeed-0.05)) finalSpeed = finalSpeed*rampSpeedIncrement + lastSpeed ; // Ramp the speed if it is increasing
        lastSpeed = finalSpeed ;

        // Telemetry. TODO: Consider moving this or disabling by default
        opMode.telemetry.addData("=> Distance ", distance);
        opMode.telemetry.addData("=>    Speed ", finalSpeed);

        return finalSpeed ;
    }

    private double calculateRotationSpeed( double angle, double maxTurnSpeed ) {
        double finalTurnSpeed = maxTurnSpeed ;

        return finalTurnSpeed ;
    }

    public boolean areWeThereYet( double distance ) {
        // If the robot is not at the destination AND it is not stuck...
        if (Math.abs(distance) > nav_stopDistanceThreshold ) lastTimeNotThere = opMode.getRuntime();
        // If we are there, check how long it has been there - to make sure we're not just passing through...
        else {
            if(opMode.getRuntime() - lastTimeNotThere > nav_stopTimeThreshold) return true ;
        }
        // TODO: Add a component for rotation
        return false ;
    }

    // -------------- Overrides ------------------
    //make these speeds negative if going wrong direction
    @Override
    protected double calculateTurnSpeed(double targetAngle, double speed){
        return super.calculateTurnSpeed(targetAngle,-speed);
    }

    @Override
    protected double[] calculateMoveAngleSpeed(double targetX, double targetY, double speed){
        return super.calculateMoveAngleSpeed(targetX, targetY, -speed);
    }

}
