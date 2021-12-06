package org.firstinspires.ftc.teamcode._RobotCode.BelindaChassis;

import org.firstinspires.ftc.teamcode.Orion.Archive.NavProfiles.OdometryProfile;

public class BelindaOdometryProfile extends OdometryProfile
{
    public static final double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS_DEAD_WHEELS = 1.0; // in
    public static double GEAR_RATIO_DEAD_WHEELS = 1.0; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.2; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6.0; // in; offset of the lateral wheel

    public static double ROT_LEFT = 180.0;
    public static double ROT_RIGHT = 0.0;
    public static double ROT_FRONT = 90.0;

    public static String LEFT_ENCODER_NAME = "FL";
    public static String RIGHT_ENCODER_NAME = "FR";
    public static String FRONT_ENCODER_NAME = "RR";

    public static boolean LEFT_ENCODER_REVERSED = false;
    public static boolean RIGHT_ENCODER_REVERSED = false;
    public static boolean FRONT_ENCODER_REVERSED = false;

    @Override
    public double TICKS_PER_REV() {return TICKS_PER_REV;}

    @Override
    public double WHEEL_RADIUS_DEAD_WHEELS() {return WHEEL_RADIUS_DEAD_WHEELS;}

    @Override
    public double GEAR_RATIO_DEAD_WHEELS() {return GEAR_RATIO_DEAD_WHEELS;}

    @Override
    public double LATERAL_DISTANCE() {return LATERAL_DISTANCE;}

    @Override
    public double FORWARD_OFFSET() {return FORWARD_OFFSET;}

    @Override
    public double ROT_LEFT() {return ROT_LEFT;}

    @Override
    public double ROT_RIGHT() {return ROT_RIGHT;}

    @Override
    public double ROT_FRONT() {return ROT_FRONT;}

    @Override
    public String LEFT_ENCODER_NAME() {return LEFT_ENCODER_NAME;}

    @Override
    public String RIGHT_ENCODER_NAME() {return RIGHT_ENCODER_NAME;}

    @Override
    public String FRONT_ENCODER_NAME() {return FRONT_ENCODER_NAME;}

    @Override
    public boolean LEFT_ENCODER_REVERSED() {return LEFT_ENCODER_REVERSED;}

    @Override
    public boolean RIGHT_ENCODER_REVERSED() {return RIGHT_ENCODER_REVERSED;}

    @Override
    public boolean FRONT_ENCODER_REVERSED() {return FRONT_ENCODER_REVERSED;}
}
