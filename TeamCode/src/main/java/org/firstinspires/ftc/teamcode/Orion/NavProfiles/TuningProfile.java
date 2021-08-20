package org.firstinspires.ftc.teamcode.Orion.NavProfiles;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

//Config for all the variables that need to be adjusted and tuned (such as PID controllers, etc)
public abstract class TuningProfile
{
    //Drive Constants
    public abstract double kV ()  ; //1.0 / rpmToVelocity(MAX_RPM)
    public abstract double kA () ;
    public abstract double kStatic () ; //0.06510

    //Standard Mecanum Drive
    public abstract PIDCoefficients TRANSLATIONAL_PID () ;
    public abstract PIDCoefficients HEADING_PID () ;
    public abstract double LATERAL_MULTIPLIER ();

    public abstract double VX_WEIGHT ();
    public abstract double VY_WEIGHT ();
    public abstract double OMEGA_WEIGHT ();

    public abstract double MAX_VEL ();
    public abstract double MAX_ACCEL ();
    public abstract double MAX_ANG_VEL ();
    public abstract double MAX_ANG_ACCEL ();

    //Standard Tracking Wheel Localizer
    public abstract double X_MULTIPLIER () ; // Multiplier in the X direction
    public abstract double Y_MULTIPLIER () ; // Multiplier in the Y direction

    public TuningProfile(){}
}
