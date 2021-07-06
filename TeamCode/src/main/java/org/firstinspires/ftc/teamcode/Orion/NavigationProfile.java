package org.firstinspires.ftc.teamcode.Orion;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

public abstract class NavigationProfile
{
    //Drive Constants
    public abstract double TICKS_PER_REV () ;
    public abstract double MAX_RPM () ;
    public abstract boolean RUN_USING_ENCODER () ;
    public abstract PIDCoefficients MOTOR_VELO_PID () ;
    public abstract double WHEEL_RADIUS_CHASSIS () ; // in
    public abstract double GEAR_RATIO_CHASSIS () ; // output (wheel) speed / input (motor) speed
    public abstract double TRACK_WIDTH () ; // in
    public abstract double kV ()  ; //1.0 / rpmToVelocity(MAX_RPM)
    public abstract double kA () ;
    public abstract double kStatic () ; //0.06510

    //Sample Mecanum Drive
    public abstract PIDCoefficients TRANSLATIONAL_PID () ;
    public abstract PIDCoefficients HEADING_PID () ;

    public abstract String frontRightName () ;
    public abstract String frontLeftName () ;
    public abstract String backRightName () ;
    public abstract String backLeftName () ;

    public abstract boolean leftReversed () ;
    public abstract boolean rightReversed () ;

    //Standard Tracking Wheel Localizer
    public abstract double WHEEL_RADIUS_DEAD_WHEELS () ; // in
    public abstract double GEAR_RATIO_DEAD_WHEELS () ; // output (wheel) speed / input (encoder) speed

    public abstract double LATERAL_DISTANCE () ; // in; distance between the left and right wheels
    public abstract double FORWARD_OFFSET () ; // in; offset of the lateral wheel

    public abstract double X_MULTIPLIER () ; // Multiplier in the X direction
    public abstract double Y_MULTIPLIER () ; // Multiplier in the Y direction

    public abstract double ROT_LEFT () ;
    public abstract double ROT_RIGHT () ;
    public abstract double ROT_FRONT () ;

    public abstract String LEFT_ENCODER_NAME () ;
    public abstract String RIGHT_ENCODER_NAME () ;
    public abstract String FRONT_ENCODER_NAME () ;

    public abstract boolean LEFT_ENCODER_REVERSED () ;
    public abstract boolean RIGHT_ENCODER_REVERSED () ;
    public abstract boolean FRONT_ENCODER_REVERSED () ;

    //TF Object Detector
    public abstract String TFOD_MODEL_ASSET () ;
    public abstract String LABEL_FIRST_ELEMENT () ;
    public abstract String LABEL_SECOND_ELEMENT () ;
    public abstract double cameraXOffset () ;

    public NavigationProfile(){

    }
}
