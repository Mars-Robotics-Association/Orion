package org.firstinspires.ftc.teamcode.Phobos.behaviors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Phobos.PhobosBehavior;

import java.util.Arrays;

public class PhobosMecanumDrive extends PhobosBehavior {

    public static class Configuration{
        protected double WHEEL_DIAMETER; // in centimeters
        protected double TICKS_PER_REVOLUTION; // encoder ticks per wheel revolution
        protected double MAX_SPEED;
        protected boolean FLIP_FL = false;
        protected boolean FLIP_FR = false;
        protected boolean FLIP_BL = false;
        protected boolean FLIP_BR = false;
    }

    private Configuration config;

    private DcMotorSimple flMotor;
    private DcMotorSimple frMotor;
    private DcMotorSimple blMotor;
    private DcMotorSimple brMotor;

    public void setCartesianSpeed(double zSpeed, double xSpeed, double rotationSpeed){
        double flSpeed = zSpeed + xSpeed + rotationSpeed; // front left motor speed
        double frSpeed = zSpeed - xSpeed - rotationSpeed; // front right motor speed
        double blSpeed = zSpeed - xSpeed + rotationSpeed; // back left motor speed
        double brSpeed = zSpeed + xSpeed - rotationSpeed; // back right motor speed

        double maxSpeed = Math.max(Math.max(Math.abs(flSpeed), Math.abs(frSpeed)), Math.max(Math.abs(blSpeed), Math.abs(brSpeed)));

        if (maxSpeed > config.MAX_SPEED) {
            flSpeed /= maxSpeed / config.MAX_SPEED;
            frSpeed /= maxSpeed / config.MAX_SPEED;
            blSpeed /= maxSpeed / config.MAX_SPEED;
            brSpeed /= maxSpeed / config.MAX_SPEED;
        }

        flMotor.setPower(flSpeed);
        frMotor.setPower(frSpeed);
        blMotor.setPower(blSpeed);
        brMotor.setPower(brSpeed);
    }

    /**
     * Called on {@link OpMode#init()}
     */
    @Override
    protected void init() throws Exception {
        DcMotorSimple[] motors = getHardwareArray(DcMotorSimple.class,
                "fl",
                "fr",
                "bl",
                "br"
        );

        flMotor = motors[0];
        frMotor = motors[1];
        blMotor = motors[2];
        brMotor = motors[3];
    }

    /**
     * Called on {@link OpMode#start()}
     */
    @Override
    protected void start() throws Exception {

    }

    /**
     * Called on {@link OpMode#loop()} or synthetically by {@linkplain LinearOpMode#runOpMode()}
     */
    @Override
    protected void update() throws Exception {

    }

    /**
     * Called on {@link OpMode#stop()}
     */
    @Override
    protected void stop() throws Exception {

    }
}
