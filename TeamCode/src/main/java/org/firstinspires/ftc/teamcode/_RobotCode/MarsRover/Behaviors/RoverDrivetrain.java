package org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._RobotCode.MarsRover.Behavior;

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
    private Configuration config;

    /**Physics properties of the robot */
    public static class Configuration{
        /**Distance from any corner wheel to the Z axis*/
        protected double cornerBaseHalf = 1;
        /**Distance from either middle wheel to the Z axis*/
        protected double centerBaseHalf = 1;
        /**Distance from the center axle to front or back axle*/
        protected double ZMiddleDistance = 1;
    }

    public RoverDrivetrain(Configuration config){
        this.config = config;
    }

    // The DCMotor class has a lot of overhead that we don't really need. Could save on battery?
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

    public void fullDrive(double driveSpeed, double turnRadius) {
        boolean turnRight = turnRadius >= 0;

        // make sure this is positive
        turnRadius = Math.abs(turnRadius);

        if((turnRadius <= config.cornerBaseHalf) || (driveSpeed == 0)){
            // what are you doing here?
            stopMoving();
            return;
        }

        double tanInner = Math.tan(config.ZMiddleDistance / (turnRadius - config.cornerBaseHalf));
        double tanOuter = -Math.tan(config.ZMiddleDistance / (turnRadius + config.cornerBaseHalf));

        setServoPos(0, turnRight ? tanOuter : tanInner);
        setServoPos(1, turnRight ? tanOuter : tanInner);

        setServoPos(2, turnRight ? tanInner : tanOuter);
        setServoPos(3, turnRight ? tanInner : tanOuter);

        double speed1PWM = 0;
        double speed2PWM = 0;
        double speed3PWM = 0;

        // I wanted to condense this using some weird byte
        // mask switch case but my brain can't process that.
        // I'm not even sure if this works at all.

        if(turnRight) { // Right
            if (driveSpeed > 0) { // Forward
                setMotorPower(0, speed1PWM);
                setMotorPower(1, speed1PWM);
                setMotorPower(2, speed1PWM);
                setMotorPower(3, -speed2PWM);
                setMotorPower(4, -speed3PWM);
                setMotorPower(5, -speed2PWM);
            }else if (driveSpeed < 0) { // Backward
                setMotorPower(0, -speed1PWM);
                setMotorPower(1, -speed1PWM);
                setMotorPower(2, -speed1PWM);
                setMotorPower(3, speed2PWM);
                setMotorPower(4, speed3PWM);
                setMotorPower(5, speed2PWM);
            }
        }else{ // Left
            if (driveSpeed > 0) { // Forward
                setMotorPower(0, speed2PWM);
                setMotorPower(1, speed3PWM);
                setMotorPower(2, speed2PWM);
                setMotorPower(3, -speed1PWM);
                setMotorPower(4, -speed1PWM);
                setMotorPower(5, -speed1PWM);
            }else if (driveSpeed < 0) { // Backward
                setMotorPower(0, -speed2PWM);
                setMotorPower(1, -speed3PWM);
                setMotorPower(2, -speed2PWM);
                setMotorPower(3, speed1PWM);
                setMotorPower(4, speed1PWM);
                setMotorPower(5, speed1PWM);
            }
        }

        /*   you know what? f*** it! lets do byte-level stupid!
            byte funnyByte = 0x00;

            // apply turn (right = 1, left = 0)
            funnyByte |= turnRight ? 0x10 : 0x00;

            // apply direction (forward = 1, backward = 0)
            funnyByte |= (driveSpeed > 0) ? 0x01 : 0x00;

            switch(funnyByte){
                case 0x11:
                    setMotorPower(0, speed1PWM);
                    setMotorPower(1, speed1PWM);
                    setMotorPower(2, speed1PWM);
                    setMotorPower(3, -speed2PWM);
                    setMotorPower(4, -speed3PWM);
                    setMotorPower(5, -speed2PWM);
                    break;
                case 0x10:
                    setMotorPower(0, -speed1PWM);
                    setMotorPower(1, -speed1PWM);
                    setMotorPower(2, -speed1PWM);
                    setMotorPower(3, speed2PWM);
                    setMotorPower(4, speed3PWM);
                    setMotorPower(5, speed2PWM);
                    break;
                case 0x01:
                    setMotorPower(0, speed2PWM);
                    setMotorPower(1, speed3PWM);
                    setMotorPower(2, speed2PWM);
                    setMotorPower(3, -speed1PWM);
                    setMotorPower(4, -speed1PWM);
                    setMotorPower(5, -speed1PWM);
                    break;
                case 0x00:
                    setMotorPower(0, -speed2PWM);
                    setMotorPower(1, -speed3PWM);
                    setMotorPower(2, -speed2PWM);
                    setMotorPower(3, speed1PWM);
                    setMotorPower(4, speed1PWM);
                    setMotorPower(5, speed1PWM);
            }
        */
    }

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

    @Override
    protected void init() {
        motors = new DcMotorSimple[]{
                hardwareMap.get(DcMotorSimple.class, "M0"),
                hardwareMap.get(DcMotorSimple.class, "M1"),
                hardwareMap.get(DcMotorSimple.class, "M2"),
                hardwareMap.get(DcMotorSimple.class, "M3"),
                hardwareMap.get(DcMotorSimple.class, "M4"),
                hardwareMap.get(DcMotorSimple.class, "M5")
        };

        servos = new Servo[]{
                hardwareMap.get(Servo.class, "S0"),
                hardwareMap.get(Servo.class, "S1"),
                hardwareMap.get(Servo.class, "S2"),
                hardwareMap.get(Servo.class, "S3")
        };

        //TODO: This is up to mechanical to decide
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[3].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[4].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[5].setDirection(DcMotorSimple.Direction.FORWARD);

        stopMoving();
    }

    @Override
    protected void start() {

    }

    @Override
    protected void update() {

    }

    @Override
    protected void stop() {
        stopMoving();
    }
}
