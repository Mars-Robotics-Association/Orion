package org.firstinspires.ftc.teamcode.Orion.NavProfiles;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//Config for all the mechanical constants of the chassis
public abstract class ChassisProfile
{
    //Drive Constants
    public abstract double MAX_RPM () ;
    public abstract boolean RUN_USING_ENCODER () ;
    public abstract PIDFCoefficients MOTOR_VELO_PID () ;
    public abstract double WHEEL_RADIUS_CHASSIS () ; // in
    public abstract double GEAR_RATIO_CHASSIS () ; // output (wheel) speed / input (motor) speed
    public abstract double TRACK_WIDTH () ; // in

    //Standard Mecanum Drive
    public abstract String frontRightName () ;
    public abstract String frontLeftName () ;
    public abstract String backRightName () ;
    public abstract String backLeftName () ;

    public abstract boolean leftReversed () ;
    public abstract boolean rightReversed () ;

    public ChassisProfile(){}

}
