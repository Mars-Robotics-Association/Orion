package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

import java.util.ArrayList;
import java.util.WeakHashMap;

public class RoverDrivetrain extends Behavior {
    /**
     * Used to ensure that all turn radii are not smaller than half of the robot's width,
     * resulting in division by zero
     */
    private double drivebaseWidth = Double.POSITIVE_INFINITY;

    public double getMinimumTurnRadius(){
        return drivebaseWidth;
    }
    DriveUnit[] driveUnits;

    /**
     * Physics properties of the robot
     */
    public static class DriveUnit {

        /**
         * Instantiates a new {@link DriveUnit}.
         *
         * @param motorName Motor name
         * @param servoName Servo name, or NULL for no turn servo.
         * @param offsetX   X Offset (-1 = left, +1 = right) in inches.
         * @param offsetY   Y Offset (-1 = back, +1 = front) in inches.
         */
        public DriveUnit(@NonNull String motorName,
                         String servoName,
                         double offsetX,
                         double offsetY){
            this.motorName = motorName;
            this.servoName = servoName;
            this.offsetX = offsetX;
            this.offsetY = offsetY;

            this.motor = hardwareMap.get(DcMotorSimple.class, motorName);
            if(servoName == null){
                this.servo = null;
            }else {
                this.servo = hardwareMap.get(Servo.class, servoName);
            }
        }

        /**
         * The DCMotor name of this unit.
         */
        public final String motorName;
        /**
         * the Servo reference of this unit, if it exists.
         */
        public final String servoName;
        /**
         * The DCMotor reference of this unit.
         */
        public final DcMotorSimple motor;
        /**
         * the Servo reference of this unit, if it exists.
         */
        public final Servo servo;
        /**
         * X Offset (-1 = left, +1 = right) in inches.
         */
        double offsetX;
        /**
         * Y Offset (-1 = back, +1 = front) in inches.
         */
        double offsetY;

        /**
         * Tells whether this drive unit has a servo.
         * @return Does this have a servo?
         */
        public boolean hasServo(){
            return servo != null;
        }

        /**
         * Convenience function for setting the motor speed
         */
        public void setSpeed(double speed) {
            motor.setPower(speed);
        }

        /**
         * Convenience function for setting the servo angle (in Radians).
         * Does nothing if {@link #servo} is NULL.
         */
        public void setRadians(double rad, boolean flipped) {
            if (servo == null){
                RobotLog.e("Drive Unit with Motor " + this.motor.getConnectionInfo());
            }

            double FIVE_OVER_SIX_PI = (5. / 6.) * Math.PI;

            double mapped = (.5 / FIVE_OVER_SIX_PI) * rad;

            assert servo != null;
            servo.setPosition(.5 + (flipped ? -1 : 1) * mapped);
        }

        public void setRadians(double rad) {
            setRadians(rad, false);
        }
    }

    /**
     * Creates a new RoverDrivetrain from the following drive units.
     */
    public RoverDrivetrain(@NonNull DriveUnit[] driveUnits) {
        this.driveUnits = driveUnits;

        for (DriveUnit unit :
                driveUnits) {
            drivebaseWidth = Math.min(drivebaseWidth, Math.abs(unit.offsetX) * 2);
        }
    }

    /**
     * Stops the robot in place. Nothing else to it.
     */
    public void stopMotors() {
        for (DriveUnit unit : driveUnits) {
            unit.setSpeed(0);
        }
    }

    /**
     * Returns the servos to their default position.
     */
    public void resetServos() {
        int i = 0;

        for (DriveUnit unit : driveUnits) {
            RobotLog.aa("Reset Servo", String.valueOf(++i));

            unit.setRadians(0);
        }
    }

    /**
     * Calls both {@link #stopMotors()} and {@link #resetServos()}.
     */
    public void stopMoving() {
        stopMotors();
        resetServos();
    }

    /**
     * Simply moves back and forth. There is literally nothing else to say here.
     */
    public void driveBackAndForth(double driveSpeed) {
        resetServos();

        for (DriveUnit unit : driveUnits) {
            unit.setSpeed(driveSpeed);
        }
    }

    private static double map(double x, double min1, double max1, double min2, double max2) {
        return (x - min1) / (max1 - min1) * (max2 - min2) + min2;
    }

    /**
     * Gives you the full power of Ackermann driving.
     * @param turnRadius Radius of a circle that is tangent to the Z axis of the robot.
     * @param angularSpeed Motor speed to move along this circle.
     */
    public void fullDrive(double turnRadius, double angularSpeed) {
        // This function won't handle zeroes nicely.
        if ((Math.abs(turnRadius) <= drivebaseWidth / 2) || (angularSpeed == 0)) {
            // what are you doing here?
            stopMoving();
            return;
        }

        for (DriveUnit driveUnit : driveUnits) {

            // Handle servo angles, if applicable
            double yPos = Math.abs(driveUnit.offsetY);
            double xPos;

            boolean rightTurn = turnRadius > 0;

            if (rightTurn) {
                xPos = turnRadius - driveUnit.offsetX;
            } else {
                xPos = -turnRadius + driveUnit.offsetX;
            }

            if (driveUnit.servo != null) {
                if(yPos == 0){
                    driveUnit.setRadians(Math.PI / 2, !rightTurn);
                }

                double tan = Math.tan(Math.abs(yPos) / xPos);

                driveUnit.setRadians(Math.abs(tan), !rightTurn);
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
     * Calculates the distance between a vector A and a vector B.
     *
     * @param x1 X component on vector A.
     * @param y1 Y component on vector A.
     * @param x2 X component on vector B.
     * @param y2 Y component on vector B.
     * @return Distance between both vectors.
     */
    private static double distance(double x1, double y1, double x2, double y2) {
        double x = x2 - x1;
        double y = y2 - y1;
        return distance(x, y);
    }

    /**
     * Spin the robot in place at a given speed
     *
     * @param angularSpeed This doesn't make much sense,
     */
    public void spinTurn(double angularSpeed) {
        for (DriveUnit driveUnit : driveUnits) {

            double fraction = driveUnit.offsetY / driveUnit.offsetX;
            driveUnit.setRadians(Math.tan(fraction), (driveUnit.offsetX < 0));

            double physicalDistance = distance(driveUnit.offsetX, driveUnit.offsetY);
            driveUnit.setSpeed(physicalDistance * angularSpeed);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void init() throws Exception {

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
