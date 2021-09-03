package org.firstinspires.ftc.teamcode._RobotCode.BelindaChassis;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Orion.NavProfiles.ChassisProfile;

public class BelindaChassisProfile extends ChassisProfile
{
    public static final double MAX_RPM = 160.0;
    public static final boolean RUN_USING_ENCODER = false;
    public static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, (32767 / (MAX_RPM / 60 * 8192)));
    public static final double WHEEL_RADIUS_CHASSIS = 2.0; // in
    public static final double GEAR_RATIO_CHASSIS = 1.0; // output (wheel) speed / input (motor) speed
    public static final double TRACK_WIDTH = 13.0; // in
    public static String frontRightName = "FR";
    public static String frontLeftName = "FL";
    public static String backRightName = "RR";
    public static String backLeftName = "RL";
    public static boolean leftReversed = false;
    public static boolean rightReversed = true;

    @Override
    public double MAX_RPM() {return MAX_RPM;}

    @Override
    public boolean RUN_USING_ENCODER() {return RUN_USING_ENCODER;}

    @Override
    public PIDFCoefficients MOTOR_VELO_PID() {return MOTOR_VELO_PID;}

    @Override
    public double WHEEL_RADIUS_CHASSIS() {return WHEEL_RADIUS_CHASSIS;}

    @Override
    public double GEAR_RATIO_CHASSIS() {return GEAR_RATIO_CHASSIS;}

    @Override
    public double TRACK_WIDTH() {return TRACK_WIDTH;}

    @Override
    public String frontRightName() {return frontRightName;}

    @Override
    public String frontLeftName() {return frontLeftName;}

    @Override
    public String backRightName() {return backRightName;}

    @Override
    public String backLeftName() {return backLeftName;}

    @Override
    public boolean leftReversed() {return leftReversed;}

    @Override
    public boolean rightReversed() {return rightReversed;}
}
