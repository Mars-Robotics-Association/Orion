package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

import java.util.ArrayList;

//Motors:

//      Front Left|Front Right
//                +
//         ^M0    |    M3^
//                |
//         ^M1    |    M4^
//                |
//         ^M2    |    M5^
//                -
//       Back Left|Back Right

//Servos:

//      Front Left|Front Right
//
//        + S0 -  |  - S2 +
//                |
//          XX    |    XX
//                |
//        + S1 -  |  - S3 +
//
//       Back Left|Back Right

public class RoverDrivetrain extends Behavior {
    ArrayList<MotorConfig> motorConfigs;

    /** Used within the {@link MotorConfig} interface to specify servo roles. */
    public enum ServoRole{
        NO_SERVO,
        LEFT_SIDE,
        RIGHT_SIDE
    }

    /**Physics properties of the robot */
    public class MotorConfig{
        /** X Offset (-1 = left, +1 = right) in meters. */
        double offsetX = 0;
        /** Z Offset (-1 = back, +1 = front) in meters. */
        double offsetY = 0;
        /** The DCMotor reference of this motor. */
        DcMotorSimple motor = null;
        /** The role of this servo. use the NO_SERVO option */
        ServoRole servoRole = null;
        /** the Servo reference of this servo, if any. */
        Servo servoName = null;
    }

    public RoverDrivetrain addWheel(
            double offsetX,
            double offsetZ,
            String motorName,
            String servoName,
            ServoRole servoRole
    ){
        motorConfigs.add(new MotorConfig() {
        });
    }

    public RoverDrivetrain(MotorConfig ...config){
        MotorConfig instance = new MotorConfig();

        

        this.motorConfigs = config;
    }

    private DcMotorSimple[] motors;
    private Servo[] servos;

    public void stopMotors(){
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);
        motors[4].setPower(0);
        motors[5].setPower(0);
    }

    public void resetServos(){
        servos[0].setPosition(.5);
        servos[1].setPosition(.5);
        servos[2].setPosition(.5);
        servos[3].setPosition(.5);
    }

    private void setServoPos(int index, double rad){
        double PI_3_OVER_4 = (Math.PI * .75);

        boolean ccw = rad < 0;

        rad = Math.abs(rad);

        double halfPos = (2*rad / (Math.PI * .75));

        servos[index].setPosition((ccw?-1:1) * halfPos + .5);
    }

    private void setMotorPower(int index, double power){
        motors[index].setPower(power);
    }

    public void stopMoving(){
        stopMotors();
        resetServos();
    }

    public void driveBackAndForth(double driveSpeed){
        motors[0].setPower(driveSpeed);
        motors[1].setPower(driveSpeed);
        motors[2].setPower(driveSpeed);
        motors[3].setPower(driveSpeed);
        motors[4].setPower(driveSpeed);
        motors[5].setPower(driveSpeed);
    }

    private static double map(double x, double min1, double max1, double min2, double max2) {
        return (x - min1) / (max1 - min1) * (max2 - min2) + min2;
    }

    enum Steering{
        LEFT,
        RIGHT
    }

    enum Driving{
        FORWARD,
        BACKWARD
    }

    private double calculateWheelAngle(double turnRadius, double offsetX, double offsetY){

    }

    public void fullDrive(double turnRadius, double angularSpeed, Steering steering, Driving driving) {
        if((turnRadius <= config.cornerBaseHalf) || (angularSpeed == 0)){
            // what are you doing here?
            stopMoving();
            return;
        }

        double innerDistanceCorner = turnRadius - config.cornerBaseHalf;
        double outerDistanceCorner = turnRadius + config.cornerBaseHalf;

        double tanInner = Math.tan(config.ZMiddleDistance / innerDistanceCorner);
        double tanOuter = -Math.tan(config.ZMiddleDistance / outerDistanceCorner);

        boolean turnRight = steering == Steering.RIGHT;

        setServoPos(0, turnRight ? tanOuter : tanInner);
        setServoPos(1, turnRight ? tanOuter : tanInner);

        setServoPos(2, turnRight ? tanInner : tanOuter);
        setServoPos(3, turnRight ? tanInner : tanOuter);

        // Motor speeds

        double innerRadiusCenter = distance(turnRadius - config.centerBaseHalf, 0);
        double outerRadiusCenter = distance(turnRadius + config.centerBaseHalf, 0);

        double innerRadiusCorner = distance(innerDistanceCorner, config.ZMiddleDistance);
        double outerRadiusCorner = distance(outerDistanceCorner, config.ZMiddleDistance);

        innerRadiusCorner *= (driving == Driving.FORWARD) ? 1 : -1;
        outerRadiusCorner *= (driving == Driving.FORWARD) ? 1 : -1;
        innerRadiusCenter *= (driving == Driving.FORWARD) ? 1 : -1;
        outerRadiusCenter *= (driving == Driving.FORWARD) ? 1 : -1;

        setMotorPower(0, angularSpeed * (turnRight ? outerRadiusCorner : innerRadiusCorner));
        setMotorPower(1, angularSpeed * (turnRight ? outerRadiusCenter : innerRadiusCenter));
        setMotorPower(2, angularSpeed * (turnRight ? outerRadiusCorner : innerRadiusCorner));

        setMotorPower(3, angularSpeed * (turnRight ? innerRadiusCorner : outerRadiusCorner));
        setMotorPower(4, angularSpeed * (turnRight ? innerRadiusCenter : outerRadiusCenter));
        setMotorPower(5, angularSpeed * (turnRight ? innerRadiusCorner : outerRadiusCorner));
    }

    /**
     * Calculates the distance between the origin (0,0) and some point (x,y).
     * Can also be used to obtain a vector's length.
     * @param x X vector component
     * @param y Y vector component
     * @return Distance
     */
    private static double distance(double x, double y){
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Calculates the distance between vector A and vector B.
     * @param x1 X component on vector A
     * @param y1 Y component on vector A
     * @param x2 X component on vector B
     * @param y2 Y component on vector B
     * @return Distance betwee both vectors.
     */
    private static double distance(double x1, double y1, double x2, double y2){
        double x = x2 - x1;
        double y = y2 - y1;
        return distance(x, y);
    }

    /**
     * Spin the robot in place at a given speed
     * @param speed
     */
    public void spinTurn(double speed){
        setServoPos(0, Math.PI*.5);
        setServoPos(0, Math.PI*.5);
        setServoPos(0, Math.PI*.5);
        setServoPos(0, Math.PI*.5);

        setMotorPower(0, speed);
        setMotorPower(1, speed);
        setMotorPower(2, speed);

        setMotorPower(3, speed);
        setMotorPower(4, speed);
        setMotorPower(5, speed);
    }

    /**
     * {@inheritDoc} Also stops the robot's motors and resets servos.
     */
    @Override
    protected void init() throws Exception {
        motors = getHardwareArray(
                DcMotorSimple.class,
                "M0",
                "M1",
                "M2",
                "M3",
                "M4",
                "M5"
        );

        servos = getHardwareArray(
                Servo.class,
                "S0",
                "S1",
                "S2",
                "S3"
        );

        //TODO: This is up to mechanical to decide
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[3].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[4].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[5].setDirection(DcMotorSimple.Direction.FORWARD);

        stopMoving();
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
