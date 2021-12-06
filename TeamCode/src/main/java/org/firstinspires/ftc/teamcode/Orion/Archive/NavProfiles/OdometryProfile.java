package org.firstinspires.ftc.teamcode.Orion.Archive.NavProfiles;

//Config for odometry
public abstract class OdometryProfile
{
    //Standard Tracking Wheel Localizer
    public abstract double TICKS_PER_REV () ;
    public abstract double WHEEL_RADIUS_DEAD_WHEELS () ; // in
    public abstract double GEAR_RATIO_DEAD_WHEELS () ; // output (wheel) speed / input (encoder) speed

    public abstract double LATERAL_DISTANCE () ; // in; distance between the left and right wheels
    public abstract double FORWARD_OFFSET () ; // in; offset of the lateral wheel
    public abstract double ROT_LEFT () ;
    public abstract double ROT_RIGHT () ;
    public abstract double ROT_FRONT () ;
    public abstract String LEFT_ENCODER_NAME () ;
    public abstract String RIGHT_ENCODER_NAME () ;
    public abstract String FRONT_ENCODER_NAME () ;

    public abstract boolean LEFT_ENCODER_REVERSED () ;
    public abstract boolean RIGHT_ENCODER_REVERSED () ;
    public abstract boolean FRONT_ENCODER_REVERSED () ;

    public OdometryProfile(){}

}
