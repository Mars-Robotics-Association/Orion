package org.firstinspires.ftc.teamcode._RobotCode.Ingenuity;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Orion.Archive.NavProfiles.NavigationProfile;
import org.firstinspires.ftc.teamcode._RobotCode.BelindaChassis.BelindaChassisProfile;
import org.firstinspires.ftc.teamcode._RobotCode.BelindaChassis.BelindaOdometryProfile;
import org.firstinspires.ftc.teamcode._RobotCode.BelindaChassis.BelindaTuningProfile;
import org.firstinspires.ftc.teamcode._RobotCode.BelindaChassis.BelindaVisionProfile;

public class IngenuityNavProfile extends NavigationProfile
{
    public IngenuityNavProfile() {
        super(new BelindaTuningProfile(), new BelindaChassisProfile(), new BelindaVisionProfile(), new BelindaOdometryProfile());
    }

    //Drive Constants
    public static final double TICKS_PER_REV = 8192;
    public static final double MAX_RPM = 160.0;
    public static final boolean RUN_USING_ENCODER = false;
    public static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, (32767 / (MAX_RPM / 60 * TICKS_PER_REV)));
    public static final double WHEEL_RADIUS_CHASSIS = 2.0; // in
    public static final double GEAR_RATIO_CHASSIS = 1.0; // output (wheel) speed / input (motor) speed
    public static final double TRACK_WIDTH = 13.0; // in
    public static final double kV =  0.01494; //1.0 / rpmToVelocity(MAX_RPM)
    public static final double kA = 0.00004;
    public static final double kStatic = 0.04877; //0.06510
    public static final double MAX_VEL = 60.0;
    public static final double MAX_ACCEL = 40.0;
    public static final double MAX_ANG_VEL = 10.0;
    public static final double MAX_ANG_ACCEL = 5.0;

    //Sample Mecanum Drive
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8.0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8.0, 0, 0);

    public static String frontRightName = "FR";
    public static String frontLeftName = "FL";
    public static String backRightName = "RR";
    public static String backLeftName = "RL";

    public static boolean leftReversed = false;
    public static boolean rightReversed = true;

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    //Standard Tracking Wheel Localizer
    public static double WHEEL_RADIUS_DEAD_WHEELS = 1.0; // in
    public static double GEAR_RATIO_DEAD_WHEELS = 1.0; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.2; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6.0; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.0; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.0; // Multiplier in the Y direction

    public static double ROT_LEFT = 180.0;
    public static double ROT_RIGHT = 0.0;
    public static double ROT_FRONT = 90.0;

    public static String LEFT_ENCODER_NAME = "FL";
    public static String RIGHT_ENCODER_NAME = "FR";
    public static String FRONT_ENCODER_NAME = "RR";

    public static boolean LEFT_ENCODER_REVERSED = false;
    public static boolean RIGHT_ENCODER_REVERSED = false;
    public static boolean FRONT_ENCODER_REVERSED = false;

    //TF Object Detector
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    public static double cameraXOffset = -400.0;


    //THESE ARE USED TO GET THE VALUES

    public double TICKS_PER_REV() {return TICKS_PER_REV;}
    public double MAX_RPM() {return MAX_RPM;}
    public boolean RUN_USING_ENCODER() {return RUN_USING_ENCODER;}
    public PIDFCoefficients MOTOR_VELO_PID() {return MOTOR_VELO_PID;}
    public double WHEEL_RADIUS_CHASSIS() {return WHEEL_RADIUS_CHASSIS;}
    public double GEAR_RATIO_CHASSIS() {return GEAR_RATIO_CHASSIS;}
    public double TRACK_WIDTH() {return TRACK_WIDTH;}
    public double kV() {return kV;}
    public double kA() {return kA;}
    public double kStatic() {return kStatic;}
    public PIDCoefficients TRANSLATIONAL_PID() {return TRANSLATIONAL_PID;}
    public PIDCoefficients HEADING_PID() {return HEADING_PID;}
    public String frontRightName() {return frontRightName;}
    public String frontLeftName() {return frontLeftName;}
    public String backRightName() {return backRightName;}
    public String backLeftName() {return backLeftName;}
    public boolean leftReversed() {return leftReversed;}
    public boolean rightReversed() {return rightReversed;}

    public double LATERAL_MULTIPLIER() {return LATERAL_MULTIPLIER;}
    public double VX_WEIGHT() {return VX_WEIGHT;}
    public double VY_WEIGHT() {return VY_WEIGHT;}
    public double OMEGA_WEIGHT() {return OMEGA_WEIGHT;}
    public double MAX_VEL() {return MAX_VEL;}
    public double MAX_ACCEL() {return MAX_ACCEL;}
    public double MAX_ANG_VEL() {return MAX_ANG_VEL;}
    public double MAX_ANG_ACCEL() {return MAX_ANG_ACCEL;}
    public double WHEEL_RADIUS_DEAD_WHEELS() {return WHEEL_RADIUS_DEAD_WHEELS;}
    public double GEAR_RATIO_DEAD_WHEELS() {return GEAR_RATIO_DEAD_WHEELS;}
    public double LATERAL_DISTANCE() {return LATERAL_DISTANCE;}
    public double FORWARD_OFFSET() {return FORWARD_OFFSET;}
    public double X_MULTIPLIER() {return X_MULTIPLIER;}
    public double Y_MULTIPLIER() {return Y_MULTIPLIER;}
    public double ROT_LEFT() {return ROT_LEFT;}
    public double ROT_RIGHT() {return ROT_RIGHT;}
    public double ROT_FRONT() {return ROT_FRONT;}
    public String LEFT_ENCODER_NAME() {return LEFT_ENCODER_NAME;}
    public String RIGHT_ENCODER_NAME() {return RIGHT_ENCODER_NAME;}
    public String FRONT_ENCODER_NAME() {return FRONT_ENCODER_NAME;}
    public boolean LEFT_ENCODER_REVERSED() {return LEFT_ENCODER_REVERSED;}
    public boolean RIGHT_ENCODER_REVERSED() {return RIGHT_ENCODER_REVERSED;}
    public boolean FRONT_ENCODER_REVERSED() {return FRONT_ENCODER_REVERSED;}
    public String TFOD_MODEL_ASSET() {return TFOD_MODEL_ASSET;}
    public String LABEL_FIRST_ELEMENT() {return LABEL_FIRST_ELEMENT;}
    public String LABEL_SECOND_ELEMENT() {return LABEL_SECOND_ELEMENT;}
    public double cameraXOffset() {return cameraXOffset;}
}
