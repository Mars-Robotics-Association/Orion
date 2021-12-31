package org.firstinspires.ftc.teamcode._RobotCode._Defaults;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Orion.NavModules.Roadrunner.RoadrunnerRobotProfile;

@Config
public class _DefaultRRRobotProfile implements RoadrunnerRobotProfile
{
    public static double TICKS_PER_REV = 537.6;
    public static double MAX_RPM = 160.0;
    public static boolean RUN_USING_ENCODER = true;

    public static double MOTOR_VELO_P = 0;
    public static double MOTOR_VELO_I = 0;
    public static double MOTOR_VELO_D = 0;
    public static double MOTOR_VELO_F = (32767 / (MAX_RPM / 60 * 8192));

    public static double WHEEL_RADIUS_CHASSIS = 3.78/2;
    public static double GEAR_RATIO_CHASSIS = 1;
    public static double TRACK_WIDTH = 11.3;

    public static String frontRightName = "FR";
    public static String frontLeftName = "FL";
    public static String backRightName = "RR";
    public static String backLeftName = "RL";

    @Override
    public double TICKS_PER_REV() {return TICKS_PER_REV;}

    @Override
    public double MAX_RPM() {return MAX_RPM;}

    @Override
    public boolean RUN_USING_ENCODER() {return RUN_USING_ENCODER;}

    @Override
    public PIDFCoefficients MOTOR_VELO_PID() {return new PIDFCoefficients(MOTOR_VELO_P,MOTOR_VELO_I,MOTOR_VELO_D,MOTOR_VELO_F);}

    @Override
    public double WHEEL_RADIUS_CHASSIS() {return WHEEL_RADIUS_CHASSIS;}

    @Override
    public double GEAR_RATIO_CHASSIS() {return GEAR_RATIO_CHASSIS; }

    @Override
    public double TRACK_WIDTH() {return TRACK_WIDTH;}

    @Override
    public String frontRightName() {return frontRightName;}

    @Override
    public String frontLeftName() {return frontLeftName;}

    @Override
    public String backRightName() {return backRightName;}

    @Override
    public String backLeftName() {return backLeftName; }
}
