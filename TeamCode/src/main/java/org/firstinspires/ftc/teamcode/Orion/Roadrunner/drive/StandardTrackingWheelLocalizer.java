package org.firstinspires.ftc.teamcode.Orion.Roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Orion.NavigationProfile;
import org.firstinspires.ftc.teamcode.Orion.Roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.0; // in
    public static double GEAR_RATIO = 1.0; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.2; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.0; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.0; // Multiplier in the Y direction

    public static double ROT_LEFT = 180.0;
    public static double ROT_RIGHT = 180.0;
    public static double ROT_FRONT = 270.0;

    public static String LEFT_ENCODER_NAME = "FL";
    public static String RIGHT_ENCODER_NAME = "FR";
    public static String FRONT_ENCODER_NAME = "RR";

    public static boolean LEFT_ENCODER_REVERSED = false;
    public static boolean RIGHT_ENCODER_REVERSED = false;
    public static boolean FRONT_ENCODER_REVERSED = false;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, NavigationProfile navProfile) {
        super(Arrays.asList(
                new Pose2d(0, navProfile.LATERAL_DISTANCE() / 2, Math.toRadians(navProfile.ROT_LEFT())), // left
                new Pose2d(0, -navProfile.LATERAL_DISTANCE() / 2, Math.toRadians(navProfile.ROT_RIGHT())), // right
                new Pose2d(navProfile.FORWARD_OFFSET(), 0, Math.toRadians(navProfile.ROT_FRONT())) // front
        ));

        TICKS_PER_REV = navProfile.TICKS_PER_REV();
        WHEEL_RADIUS = navProfile.WHEEL_RADIUS_DEAD_WHEELS();
        GEAR_RATIO = navProfile.GEAR_RATIO_DEAD_WHEELS();
        LATERAL_DISTANCE = navProfile.LATERAL_DISTANCE();
        FORWARD_OFFSET = navProfile.FORWARD_OFFSET();
        X_MULTIPLIER = navProfile.X_MULTIPLIER();
        Y_MULTIPLIER = navProfile.Y_MULTIPLIER();
        ROT_LEFT = navProfile.ROT_LEFT();
        ROT_RIGHT = navProfile.ROT_RIGHT();
        ROT_FRONT = navProfile.ROT_FRONT();
        LEFT_ENCODER_NAME = navProfile.LEFT_ENCODER_NAME();
        RIGHT_ENCODER_NAME = navProfile.RIGHT_ENCODER_NAME();
        FRONT_ENCODER_NAME = navProfile.FRONT_ENCODER_NAME();

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LEFT_ENCODER_NAME));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RIGHT_ENCODER_NAME));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, FRONT_ENCODER_NAME));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        if(LEFT_ENCODER_REVERSED) leftEncoder.setDirection(Encoder.Direction.REVERSE);
        if(RIGHT_ENCODER_REVERSED) rightEncoder.setDirection(Encoder.Direction.REVERSE);
        if(FRONT_ENCODER_REVERSED) frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
