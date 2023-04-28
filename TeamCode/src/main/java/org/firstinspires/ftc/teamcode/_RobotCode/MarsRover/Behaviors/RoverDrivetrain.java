package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

import java.util.ArrayList;
import java.util.WeakHashMap;

public class RoverDrivetrain extends Behavior {

    private final Configurator configurator;
    private double drivebaseWidth = Double.POSITIVE_INFINITY;
    ArrayList<DriveUnit> driveUnits;

    public interface Configurator {
        void applySettings(RoverDrivetrain drivetrain);
    }

    /**
     * Physics properties of the robot
     */
    public static class DriveUnit {
        /**
         * The DCMotor reference of this motor.
         */
        DcMotorSimple motor = null;
        /**
         * the Servo reference of this servo, if it exists.
         */
        Servo servo = null;
        /**
         * X Offset (-1 = left, +1 = right) in centimeters.
         */
        double offsetX = 0;
        /**
         * Y Offset (-1 = back, +1 = front) in centimeters.
         */
        double offsetY = 0;


        /**
         * Convenience function for setting the motor speed
         */
        protected void setSpeed(double speed) {
            motor.setPower(0);
        }

        /**
         * Convenience function for setting the servo angle (in Radians).
         * Does nothing if {@link #servo} is NULL.
         */
        protected void setRadians(double rad) {
            if (servo == null) return;

            double PI_3_OVER_4 = (Math.PI * .75);

            boolean ccw = rad < 0;

            rad = Math.abs(rad);

            double halfPos = (2 * rad / (Math.PI * .75));

            servo.setPosition((ccw ? -1 : 1) * halfPos + .5);
        }
    }

    /**
     * Registers a new {@link DriveUnit}. Used within a {@link Configurator} class.
     *
     * @param motorName Motor name
     * @param servoName Servo name, or NULL for no turn servo.
     * @param offsetX   X Offset (-1 = left, +1 = right) in meters.
     * @param offsetY   Y Offset (-1 = back, +1 = front) in meters.
     * @return This drivetrain for method chaining.
     */
    public RoverDrivetrain addWheel(
            @NonNull String motorName,
            String servoName,
            double offsetX,
            double offsetY
    ) {
        DriveUnit unit = new DriveUnit();
        unit.offsetX = offsetX;
        unit.offsetY = offsetY;
        unit.motor = hardwareMap.get(DcMotorSimple.class, motorName);
        assert unit.motor != null;
        if (servoName == null) return this;
        unit.servo = hardwareMap.get(Servo.class, servoName);
        assert unit.servo != null;
        return this;
    }

    /**
     * Takes in a {@link Configurator} to apply drivetrain settings.
     *
     * @param configurator The {@link Configurator} to pull settings from.
     */
    public RoverDrivetrain(@NonNull Configurator configurator) {
        this.configurator = configurator;
    }

    public void stopMotors() {
        for (DriveUnit unit : driveUnits) {
            unit.setSpeed(0);
        }
    }

    public void resetServos() {
        for (DriveUnit unit : driveUnits) {
            unit.setRadians(0);
        }
    }

    public void stopMoving() {
        stopMotors();
        resetServos();
    }

    public void driveBackAndForth(double driveSpeed) {
        resetServos();

        for (DriveUnit unit : driveUnits) {
            unit.setSpeed(driveSpeed);
        }
    }

    private static double map(double x, double min1, double max1, double min2, double max2) {
        return (x - min1) / (max1 - min1) * (max2 - min2) + min2;
    }

    public void fullDrive(double turnRadius, double angularSpeed) {

        // This function won't handle zeroes nicely.
        if ((turnRadius <= drivebaseWidth / 2) || (angularSpeed == 0)) {
            // what are you doing here?
            stopMoving();
            return;
        }

        for (DriveUnit driveUnit : driveUnits) {

            // Handle servo angles, if applicable

            double yPos = driveUnit.offsetY;
            double xPos = 0;

            if (turnRadius > 0) {
                xPos = turnRadius - driveUnit.offsetX;
            } else {
                xPos = -turnRadius + driveUnit.offsetX;

            }

            if (driveUnit.servo != null) {
                double tan = Math.tan(Math.abs(yPos) / xPos);
                driveUnit.setRadians(tan);
            }

            double physicalRadius = distance(xPos, yPos);

            driveUnit.setSpeed(physicalRadius * angularSpeed);
        }
    }

    /**
     * Calculates the distance between the origin (0,0) and some point (x,y).
     * Can also be used to obtain a vector's length.
     *
     * @param x X vector component
     * @param y Y vector component
     * @return Distance
     */
    private static double distance(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Calculates the distance between vector A and vector B.
     *
     * @param x1 X component on vector A
     * @param y1 Y component on vector A
     * @param x2 X component on vector B
     * @param y2 Y component on vector B
     * @return Distance betwee both vectors.
     */
    private static double distance(double x1, double y1, double x2, double y2) {
        double x = x2 - x1;
        double y = y2 - y1;
        return distance(x, y);
    }

    /**
     * Spin the robot in place at a given speed
     *
     * @param angularSpeed This doesn't make much sense
     */
    public void spinTurn(double angularSpeed) {
        for (DriveUnit driveUnit : driveUnits) {

            double frac = driveUnit.offsetY / driveUnit.offsetX;
            driveUnit.setRadians(Math.tan(frac));

            double physicalDistance = distance(driveUnit.offsetX, driveUnit.offsetY);
            driveUnit.setSpeed(physicalDistance * angularSpeed);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void init() throws Exception {
        configurator.applySettings(this);

        for (DriveUnit unit :
                driveUnits) {
            drivebaseWidth = Math.min(drivebaseWidth, Math.abs(unit.offsetX) * 2);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void start() {

    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void update() {

    }

    /**
     * {@inheritDoc}. Also stops the robot's motors and resets servos.
     */
    @Override
    protected void stop() {
        stopMoving();
    }
}
